from excavator_safety.paths import ROOT
import json, time, urllib.request, subprocess
from pathlib import Path

base = "http://127.0.0.1:8080"
out = Path(ROOT / "evidence/cockpit_os")
out.mkdir(parents=True, exist_ok=True)


def state():
    return json.load(urllib.request.urlopen(base + "/api/state", timeout=5))


def post(path, payload):
    req = urllib.request.Request(
        base + path,
        data=json.dumps(payload).encode(),
        headers={
            "Content-Type": "application/json",
            "X-Local-Control": state()["control_token"],
        },
    )
    return json.load(urllib.request.urlopen(req, timeout=8))


def ready():
    end = time.monotonic() + 65
    while time.monotonic() < end:
        s = state()
        if s["simulation"]["job"]["state"] == "error":
            raise AssertionError(s["simulation"])
        if (
            s["simulation"]["job"]["state"] == "success"
            and s["status"]
            and s["status"]["healthy"]
            and s["panorama"]["online"]
        ):
            return s
        time.sleep(0.2)
    raise AssertionError("not ready")


for _ in range(50):
    try:
        state()
        break
    except OSError:
        time.sleep(0.2)
post("/api/action", {"action": "start"})
ready()
records = []
for i in range(3):
    post("/api/action", {"action": "reset"})
    s = ready()
    assert s["status"]["requested"] == 0 and abs(s["status"]["measured_speed"]) < 0.001
    records.append(
        {
            "cycle": i + 1,
            "status": s["status"],
            "birdseye": s["panorama"],
            "job": s["simulation"]["job"],
        }
    )
    print("RESET", i + 1, "passed", flush=True)
(out / "reset_regression.json").write_text(
    json.dumps({"passed": True, "cycles": records}, indent=2)
)
r = subprocess.run(
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
        'name: "target_0" position: {x: 3.0 y: 0.3 z: 0} orientation: {w: 1}',
    ],
    capture_output=True,
    text=True,
    timeout=8,
)
assert "true" in r.stdout
time.sleep(1)
post("/api/drive", {"speed": 1.0})
end = time.monotonic() + 10
while time.monotonic() < end:
    s = state()
    if s["status"]["aeb_active"] and abs(s["status"]["measured_speed"]) < 0.001:
        break
    time.sleep(0.2)
else:
    raise AssertionError("AEB UI fixture failed")
(out / "aeb_ui_state.json").write_text(json.dumps(s["status"], indent=2))
print("AEB UI fixture ready; stationary with protection active", flush=True)
