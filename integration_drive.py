"""VM integration validation. Scene poses are test fixtures only, never perception inputs."""

import json, time, urllib.request, subprocess, os
from pathlib import Path

BASE = "http://127.0.0.1:8080"
out = Path("evidence/cockpit_drive")
out.mkdir(parents=True, exist_ok=True)


def state():
    return json.load(urllib.request.urlopen(BASE + "/api/state", timeout=5))


def post(path, data):
    req = urllib.request.Request(
        BASE + path,
        data=json.dumps(data).encode(),
        headers={
            "Content-Type": "application/json",
            "X-Local-Control": state()["control_token"],
        },
    )
    return json.load(urllib.request.urlopen(req, timeout=8))


def wait(predicate, timeout=60):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s = state()
        if predicate(s):
            return s
        time.sleep(0.2)
    raise RuntimeError("timeout waiting for condition")


gap = float(os.environ.get("TEST_TARGET_GAP", "9"))
suffix = "" if gap == 9 else "_early"
records = []
try:
    s = wait(
        lambda s: s["simulation"]["job"]["state"] != "running"
        and s["status"]
        and s["status"]["healthy"]
    )
    post("/api/drive", {"speed": 0.0})
    wait(
        lambda s: s["status"]["requested"] == 0
        and abs(s["status"]["measured_speed"]) < 0.03
    )
    post("/api/action", {"action": "randomize", "people": 0, "obstacles": 0})
    wait(lambda s: s["simulation"]["job"]["state"] != "running")
    assert state()["simulation"]["job"]["state"] == "success"
    wait(lambda s: s["status"]["distance"] > 50)
    post("/api/drive", {"speed": 3.0})
    s = wait(lambda s: s["status"]["measured_speed"] > 2.9, 70)
    cruise = s["status"]
    print("Cruise:", cruise, flush=True)
    # Pose read is exclusively ground truth for placement of the test fixture.
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("brake_test_fixture")
    pose = []
    node.create_subscription(
        Odometry,
        "/safety/odom",
        lambda m: pose.append((m.pose.pose.position.x, m.pose.pose.position.y)),
        qos_profile_sensor_data,
    )
    end = time.monotonic() + 5
    while not pose and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert pose
    x, y = pose[-1]
    node.destroy_node()
    rclpy.shutdown()
    req = f'name: "target_0" position: {{x: {x+gap} y: {y} z: 0}} orientation: {{w: 1}}'
    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            "/world/safety/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            req,
        ],
        capture_output=True,
        text=True,
        timeout=8,
    )
    assert "true" in result.stdout
    end = time.monotonic() + 35
    first_brake = None
    first_seen = None
    while time.monotonic() < end:
        s = state()
        r = s["status"]
        records.append(r)
        if r["distance"] < 30 and first_seen is None:
            first_seen = r
        if r["brake"] and first_brake is None:
            first_brake = r
        if first_brake and abs(r["measured_speed"]) < 0.001 and r["command"] < 0.01:
            break
        time.sleep(0.1)
    else:
        raise AssertionError("did not stop")
    assert first_seen and first_brake
    assert r["distance"] > 0.5 and r["healthy"]
    summary = dict(
        passed=True,
        cruise_speed=cruise["measured_speed"],
        first_detection=first_seen,
        first_brake=first_brake,
        stopped=r,
        brake_to_stop_sim_s=r["stamp"] - first_brake["stamp"],
        source="RGB-D detections; odometry for speed feedback; world pose used only by test fixture",
    )
    (out / ("dynamic_braking" + suffix + ".json")).write_text(
        json.dumps(summary, indent=2)
    )
    (out / ("drive_samples" + suffix + ".json")).write_text(
        json.dumps(records, indent=2)
    )
    print(json.dumps(summary, indent=2), flush=True)
finally:
    post("/api/drive", {"speed": 0.0})
    post("/api/action", {"action": "reset"})
    s = wait(
        lambda s: s["simulation"]["job"]["state"] != "running"
        and s["status"]
        and s["status"]["healthy"],
        70,
    )
    assert s["status"]["requested"] == 0 and abs(s["status"]["measured_speed"]) < 0.03
    (out / "restored.json").write_text(
        json.dumps(
            {"passed": True, "status": s["status"], "birdseye": s["panorama"]}, indent=2
        )
    )
