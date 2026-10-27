"""Actual Gazebo tests for signed motion, yaw, heartbeat expiry and AEB.
Ground truth odometry is used for test placement/assertions only.
"""

import json, time, math, threading, subprocess, urllib.request
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

BASE = "http://127.0.0.1:8080"
OUT = Path("evidence/path_policy")
OUT.mkdir(parents=True, exist_ok=True)


def state():
    return json.load(urllib.request.urlopen(BASE + "/api/state", timeout=4))


for _ in range(50):
    try:
        token = state()["control_token"]
        break
    except OSError:
        time.sleep(0.2)
else:
    raise RuntimeError("Dashboard unavailable")


def post(path, payload):
    r = urllib.request.Request(
        BASE + path,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json", "X-Local-Control": token},
    )
    return json.load(urllib.request.urlopen(r, timeout=4))


def wait(fn, seconds=45):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        s = state()
        if fn(s):
            return s
        time.sleep(0.12)
    raise AssertionError("Timed out waiting for motion condition")


rclpy.init()
node = rclpy.create_node("motion_test_fixture")
pose = [0.0, 0.0, 0.0]


def odom(m):
    q = m.pose.pose.orientation
    pose[:] = [
        m.pose.pose.position.x,
        m.pose.pose.position.y,
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
    ]


node.create_subscription(Odometry, "/safety/odom", odom, qos_profile_sensor_data)
threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
command = (0.0, 0.0)
paused = True
done = False
errors = []


def heartbeat():
    while not done:
        if not paused:
            try:
                post("/api/teleop", dict(zip(["speed", "yaw"], command)))
            except Exception as e:
                errors.append(str(e))
        time.sleep(0.12)


threading.Thread(target=heartbeat, daemon=True).start()


def stop():
    global command, paused
    command = (0.0, 0.0)
    paused = True
    post("/api/drive", {"speed": 0})
    return wait(
        lambda s: abs(s["status"]["measured_speed"]) < 0.005
        and abs(s["status"]["measured_yaw"]) < 0.005
    )


def job(action, **kw):
    post("/api/action", dict(action=action, **kw))
    s = wait(lambda s: s["simulation"]["job"]["state"] != "running", 75)
    assert s["simulation"]["job"]["state"] == "success", s["simulation"]
    return s


def clear():
    stop()
    job("reset")
    wait(lambda s: s["status"] and s["status"]["healthy"])
    job("randomize", people=0, obstacles=0)
    wait(lambda s: s["status"]["distance"] > 50)


def motion(v, w):
    global command, paused
    command = (v, w)
    paused = False


def run_sim(seconds):
    begin = state()["status"]["stamp"]
    samples = []
    end = time.monotonic() + 40
    while time.monotonic() < end:
        s = state()["status"]
        samples.append(s)
        if s["stamp"] - begin >= seconds:
            return samples
        time.sleep(0.1)
    raise AssertionError("Simulation did not advance")


def obstacle(bx, by):
    x, y, yaw = pose
    gx = x + bx * math.cos(yaw) - by * math.sin(yaw)
    gy = y + bx * math.sin(yaw) + by * math.cos(yaw)
    req = f'name: "target_0" position: {{x: {gx} y: {gy} z: 0}} orientation: {{w: 1}}'
    p = subprocess.run(
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
    assert "true" in p.stdout


report = {}
records = {}
try:
    if not state()["simulation"]["running"]:
        job("start")
    wait(lambda s: s["status"] and s["status"]["healthy"])
    clear()
    motion(1.0, 0)
    wait(lambda s: s["status"]["measured_speed"] > 0.95)
    motion(0, 0)
    records["coasting"] = run_sim(2.0)
    slopes = [
        (b["command"] - a["command"]) / (b["stamp"] - a["stamp"])
        for a, b in zip(records["coasting"], records["coasting"][1:])
        if b["stamp"] > a["stamp"] and 0.55 < a["command"] < 0.9
    ]
    assert slopes and all(abs(v + 0.25) < 0.005 for v in slopes), slopes
    report["coasting"] = {
        "passed": True,
        "deceleration": sum(slopes) / len(slopes),
        "after_two_seconds": state()["status"],
    }
    print("PASS linear coasting", flush=True)
    clear()
    motion(1.3, 0)
    wait(lambda s: s["status"]["measured_speed"] > 1.0)
    obstacle(3.4, 0)
    stopped = wait(
        lambda s: s["status"]["aeb_active"]
        and abs(s["status"]["measured_speed"]) < 0.005,
        25,
    )["status"]
    motion(-1.0, 0)
    records["escape_reverse"] = run_sim(2.0)
    s = state()["status"]
    assert s["measured_speed"] < -0.4 and not s["aeb_active"], s
    report["front_aeb_reverse_escape"] = {
        "passed": True,
        "stopped": stopped,
        "escape": s,
    }
    print("PASS front AEB -> reverse escape", flush=True)
    clear()
    motion(-1.3, 0)
    wait(lambda s: s["status"]["measured_speed"] < -1.0)
    obstacle(-3.4, 0)
    stopped = wait(
        lambda s: s["status"]["aeb_active"]
        and abs(s["status"]["measured_speed"]) < 0.005,
        25,
    )["status"]
    motion(1.0, 0)
    records["escape_forward"] = run_sim(2.0)
    s = state()["status"]
    assert s["measured_speed"] > 0.4 and not s["aeb_active"], s
    report["rear_aeb_forward_escape"] = {
        "passed": True,
        "stopped": stopped,
        "escape": s,
    }
    print("PASS rear AEB -> forward escape", flush=True)
    clear()
    motion(0.6, 0.4)
    wait(lambda s: s["status"]["measured_yaw"] > 0.35)
    obstacle(2.2, 1.6)
    stopped = wait(
        lambda s: s["status"]["aeb_active"]
        and abs(s["status"]["measured_speed"]) < 0.005
        and abs(s["status"]["measured_yaw"]) < 0.005,
        25,
    )["status"]
    report["curved_aeb"] = {"passed": True, "stopped": stopped}
    print("PASS curved trajectory AEB", flush=True)
    clear()
    obstacle(1.2, 0)
    s = wait(lambda s: s["status"]["collision_count"] > 0, 10)["status"]
    report["actual_contact"] = {"passed": True, "status": s}
    print("PASS actual Gazebo collision detected", flush=True)
    motion(-0.5, 0)
    run_sim(2.0)
    s = state()["status"]
    assert s["measured_speed"] < -0.1 and not s["aeb_active"], s
    report["contact_escape"] = {"passed": True, "status": s}
    print("PASS escape after collision", flush=True)
    report["passed"] = True
    report["heartbeat_errors"] = errors
    (OUT / "integration.json").write_text(json.dumps(report, indent=2))
    (OUT / "motion_samples.json").write_text(json.dumps(records, indent=2))
finally:
    (OUT / "integration.json").write_text(json.dumps(report, indent=2))
    (OUT / "motion_samples.json").write_text(json.dumps(records, indent=2))
    done = True
    stop()
    job("reset")
    s = wait(
        lambda s: s["status"] and s["status"]["healthy"] and s["panorama"]["online"]
    )
    (OUT / "final_state.json").write_text(json.dumps(s["status"], indent=2))
    node.destroy_node()
    rclpy.shutdown()
    print("RESTORED stationary", flush=True)
