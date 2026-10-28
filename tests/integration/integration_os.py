from excavator_safety.paths import ROOT

"""End-to-end validation of calibrated cameras, 80-target scenes and AEB.
Ground truth is used exclusively for fixtures/assertions, never sent to perception.
"""

import json, time, urllib.request, subprocess, math
from pathlib import Path

BASE = "http://127.0.0.1:8080"
OUT = Path(ROOT / "evidence/cockpit_os")
OUT.mkdir(parents=True, exist_ok=True)


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


def wait(fn, timeout=60):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s = state()
        if fn(s):
            return s
        time.sleep(0.15)
    raise RuntimeError("condition timed out")


def job(action, **kw):
    wait(lambda s: s["simulation"]["job"]["state"] != "running")
    post("/api/action", dict(action=action, **kw))
    s = wait(lambda s: s["simulation"]["job"]["state"] != "running", 70)
    assert s["simulation"]["job"]["state"] == "success", s["simulation"]
    return s


def move(positions):
    req = " ".join(
        f'pose: {{name: "target_{i}" position: {{x: {x} y: {y} z: 0}} orientation: {{w: 1}}}}'
        for i, x, y in positions
    )
    r = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            "/world/safety/set_pose_vector",
            "--reqtype",
            "gz.msgs.Pose_V",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            req,
        ],
        text=True,
        capture_output=True,
        timeout=8,
    )
    assert r.returncode == 0 and "true" in r.stdout, r.stdout + r.stderr


report = {}
samples = []
try:
    wait(lambda s: s["status"] and s["status"]["healthy"])
    post("/api/drive", {"speed": 0})
    wait(lambda s: abs(s["status"]["measured_speed"]) < 0.001)
    job("randomize", people=0, obstacles=0)
    checks = []
    for k in range(8):
        i = k % 2
        angle = (k % 4) * math.pi / 2
        r = 4.0 if k < 4 else 8.0
        x, y = r * math.cos(angle), r * math.sin(angle)
        move([(1 - i, -350, 80), (i, x, y)])

        def seen(s):
            return any(
                t["label"] == ("person" if i == 0 else "obstacle")
                and math.hypot(t["position"][0] - x, t["position"][1] - y) < 0.8
                for v in s["detections"].values()
                for t in v["targets"]
            )

        s = wait(seen, 8)
        checks.append(
            dict(
                bearing=k % 4,
                range=r,
                label="person" if i == 0 else "obstacle",
                passed=True,
                observed=s["status"]["distance"],
            )
        )
    report["camera_regression"] = checks
    print("CAMERAS: 8/8 directional fixtures detected", flush=True)
    s = job("randomize", people=40, obstacles=40)
    layout = json.loads(
        Path(ROOT / "data/web_scenarios.jsonl").read_text().splitlines()[-1]
    )
    targets = [t for t in layout["targets"] if t["active"]]
    separation = min(
        math.hypot(a["x"] - b["x"], a["y"] - b["y"])
        for i, a in enumerate(targets)
        for b in targets[i + 1 :]
    )
    assert len(targets) == 80 and separation > 1.6
    assert "targets" not in s["simulation"]["layout"]
    report["layout"] = dict(
        active=len(targets),
        min_separation=separation,
        seed=layout["seed"],
        max_forward=max(t["x"] for t in targets),
    )
    (OUT / "fixture_layout.json").write_text(json.dumps(layout, indent=2))
    print("SCENE: 80 active targets; starting 60 m drive", flush=True)
    post("/api/drive", {"speed": 3.0})
    start = state()["travel_m"]
    end = time.monotonic() + 180
    while time.monotonic() < end:
        s = state()
        samples.append(dict(travel=s["travel_m"], **s["status"]))
        if s["travel_m"] - start >= 60:
            break
        assert not s["status"][
            "aeb_active"
        ], "Unexpected early blockage in clear approach"
        time.sleep(0.2)
    else:
        raise AssertionError("60 m drive not completed")
    post("/api/drive", {"speed": 0})
    s = wait(lambda s: abs(s["status"]["measured_speed"]) < 0.001)
    report["long_drive"] = dict(
        passed=True,
        travel_m=s["travel_m"] - start,
        max_speed=max(r["measured_speed"] for r in samples),
    )
    print("DRIVE:", report["long_drive"], flush=True)
    job("randomize", people=0, obstacles=0)
    wait(lambda s: s["status"]["distance"] > 50)
    post("/api/drive", {"speed": 3.0})
    s = wait(lambda s: s["status"]["measured_speed"] > 2.9, 70)
    move([(0, s["travel_m"] + 8.0, 0.25)])
    trigger = wait(lambda s: s["status"]["aeb_active"], 15)["status"]
    stopped = wait(
        lambda s: s["status"]["aeb_active"]
        and abs(s["status"]["measured_speed"]) < 0.001,
        20,
    )
    assert stopped["status"]["distance"] > 0.25
    report["aeb"] = dict(
        passed=True,
        trigger=trigger,
        stopped=stopped["status"],
        elapsed_sim_s=stopped["status"]["stamp"] - trigger["stamp"],
    )
    report["birdseye"] = stopped["panorama"]
    report["passed"] = True
    (OUT / "validation.json").write_text(json.dumps(report, indent=2))
    (OUT / "drive_samples.json").write_text(json.dumps(samples))
    print(
        "AEB_HOLD: screenshot window 18 seconds", json.dumps(report["aeb"]), flush=True
    )
    time.sleep(18)
finally:
    post("/api/drive", {"speed": 0})
    job("reset")
    s = wait(
        lambda s: s["status"] and s["status"]["healthy"] and s["panorama"]["online"], 30
    )
    (OUT / "final_state.json").write_text(
        json.dumps({k: s[k] for k in ["status", "panorama", "simulation"]}, indent=2)
    )
    print("RESTORED: healthy, zero speed", flush=True)
