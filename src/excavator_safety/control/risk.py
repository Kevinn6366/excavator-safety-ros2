"""Trajectory-dependent collision risk from sensor detections."""

import math
import numpy as np
from excavator_safety.perception.geometry import clearance


def assess_path(targets, velocity, yaw_rate, requested, requested_yaw):
    """Sensor detections vs swept body rectangles, in the live vehicle frame.
    Evaluate both current momentum and intended curvature. Receding obstacles do
    not block escape. Arc distance, closing speed and TTC use measured motion;
    intended paths keep an AEB hold stable at zero until the driver changes course.
    """
    radius = math.hypot(1.3, 0.9)
    current = abs(velocity) + radius * abs(yaw_rate)
    candidates = []
    for v, w in [(velocity, yaw_rate), (requested, requested_yaw)]:
        sweep = abs(v) + radius * abs(w)
        if sweep < 0.015:
            continue
        horizon = max(4.0, min(30.0, 10.0 / sweep))
        ts = np.linspace(0, horizon, 161)
        theta = w * ts
        if abs(w) > 0.0001:
            cx = v / w * np.sin(theta)
            cy = v / w * (1 - np.cos(theta))
        else:
            cx = v * ts
            cy = np.zeros_like(ts)
        c = np.cos(theta)
        s = np.sin(theta)
        for target in targets:
            px, py = target["position"][:2]
            r = 0.28 if target["label"] == "person" else 0.45
            dx = px - cx
            dy = py - cy
            rx = c * dx + s * dy
            ry = -s * dx + c * dy
            sep = clearance(rx, ry) - r
            # Ignore trajectories that move away, even if currently in contact.
            if sep.min() >= sep[0] - 0.025:
                continue
            hits = np.flatnonzero((sep <= 0.2) & (sep < sep[0] - 0.025))
            if not len(hits):
                continue
            idx = int(hits[0])
            travel = sweep * ts[idx]
            candidates.append(
                dict(
                    distance=float(travel),
                    side=target["side"],
                    label=target["label"],
                    separation=float(sep[0]),
                    closing_speed=float(
                        max(
                            0.0,
                            (sep[0] - sep[min(idx, 4)]) / max(ts[min(idx, 4)], 0.01),
                        )
                    ),
                )
            )
    if not candidates:
        return dict(distance=99.0, ttc=None, closing_speed=0.0, side=None, label=None)
    risk = min(candidates, key=lambda x: x["distance"])
    risk["ttc"] = risk["distance"] / current if current > 0.03 else None
    return risk
