"""Numeric evaluation only; does not draw figures or annotated images."""

from excavator_safety.paths import ROOT

import json, glob, math
from pathlib import Path
import numpy as np
from excavator_safety.perception.detection import detect
from excavator_safety.perception.geometry import unproject, clearance
from excavator_safety.control.controller import level

P = ROOT
D = P / "data"
rows = [json.loads(x) for x in (D / "events.jsonl").read_text().splitlines()]
det = [r for r in rows if r["kind"] == "detection"]
ctl = [r for r in rows if r["kind"] == "control"]
report = {
    "frames_saved": len(list(D.glob("frame_*.npz"))),
    "processing_ms_p95": (
        float(np.percentile([r["processing_ms"] for r in det], 95)) if det else None
    ),
    "control_samples": len(ctl),
    "healthy_samples": sum(r["healthy"] for r in ctl),
    "scope": "synthetic color-coded targets; not real pedestrian accuracy",
}
if (D / "scenarios.jsonl").exists():
    trials = [json.loads(x) for x in (D / "scenarios.jsonl").read_text().splitlines()]
    tp = fn = fp = correct = false_alarm = miss_alarm = 0
    lat = []
    for t in trials:
        gt = t["targets"][0]
        vx, vy = t["vehicle_xy"]
        gx, gy = gt["x"] - vx, gt["y"] - vy
        observations = [
            o
            for r in det
            if t["stamp_start"] <= r["stamp"] <= t["stamp_end"]
            for o in r["targets"]
        ]
        matches = [
            o
            for o in observations
            if o["label"] == gt["label"]
            and np.linalg.norm(np.array(o["position"][:2]) - [gx, gy]) < 1.0
        ]
        tp += bool(matches)
        fn += not bool(matches)
        fp += sum(o not in matches for o in observations)
        # Exact nearest clearance of an axis-aligned box to the rectangular vehicle footprint.
        d = math.hypot(
            max(abs(gx) - gt["half"] - 1.3, 0), max(abs(gy) - gt["half"] - 0.9, 0)
        )
        truth = level(max(0, d - t["status"].get("distance_margin", 0)))
        measured = ["SAFE", "NOTICE", "WARNING", "DANGER"].index(
            t["status"].get("proximity_state", t["status"]["state"])
        )
        correct += measured == truth
        false_alarm += measured > truth
        miss_alarm += measured < truth
        times = [
            r["stamp"] - t.get("exposure_stamp", t["stamp_start"])
            for r in ctl
            if t.get("exposure_stamp", t["stamp_start"]) <= r["stamp"] <= t["stamp_end"]
            and r["healthy"]
            and r.get("proximity_state", r["state"])
            == ["SAFE", "NOTICE", "WARNING", "DANGER"][truth]
        ]
        if times:
            lat.append(min(times))
    report.update(
        reference_policy="static proximity classification only: exact box clearance minus margin; directional AEB validated separately",
        trials=len(trials),
        trial_detected=tp,
        trial_missed=fn,
        trial_detection_rate=tp / max(1, tp + fn),
        unmatched_detection_observations=fp,
        level_correct_rate=correct / max(1, len(trials)),
        overwarning_trials=false_alarm,
        underwarning_trials=miss_alarm,
        response_observed_upper_bound_sim_seconds_p95=(
            float(np.percentile(lat, 95)) if lat else None
        ),
        response_definition="last ROS clock observed before synchronous set_pose calls to first healthy correct state; includes CLI service time and stale clock; an upper-bound observation, not end-to-end latency",
    )
errors = []
changed = total = 0
for file in sorted(D.glob("frame_*.npz"))[::8]:
    a = np.load(file)
    side = str(a["side"])
    d = a["depth"]
    p = unproject(d, a["K"], side)
    q = unproject(d, a["K"], side, pitch=0.65 + math.radians(1), offset=0.02)
    ok = np.isfinite(d) & (p[:, :, 2] > 0.18) & (p[:, :, 2] < 3.5)
    if not ok.any():
        continue
    dis = clearance(p[:, :, 0][ok], p[:, :, 1][ok])
    pert = clearance(q[:, :, 0][ok], q[:, :, 1][ok])
    errors.extend(np.abs(dis - pert)[::10].tolist())
    total += len(dis)
    changed += sum(
        np.count_nonzero((dis <= x) != (pert <= x))
        for x in [
            1.5 + ctl[-1].get("distance_margin", 0),
            3 + ctl[-1].get("distance_margin", 0),
            6 + ctl[-1].get("distance_margin", 0),
        ]
    )
report["calibration"] = {
    "perturbation": "pitch +1 degree; base x translation +2 cm",
    "distance_error_p95_m": float(np.percentile(errors, 95)) if errors else None,
    "boundary_crossings_per_valid_point": changed / max(1, total),
    "note": "visible nonground points; multiple boundary crossings can count more than once",
}
(D / "metrics.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
