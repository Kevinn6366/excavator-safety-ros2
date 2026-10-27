"""Pure perception/control routines: all distances in metres, base x forward/y left/z up."""

import math
import cv2
import numpy as np

MAX_SPEED = 50.0 / 3.6
SIDES = {"front": 0.0, "left": math.pi / 2, "rear": math.pi, "right": -math.pi / 2}
CAMERA_POSITIONS = {
    "front": (1.5, 0.55, 2.8),
    "left": (0.0, 1.0, 2.8),
    "rear": (-1.2, 0.0, 2.8),
    "right": (0.0, -1.0, 2.8),
}


def transform(side, pitch=0.65, offset=0.0):
    yaw = SIDES[side]
    c, s = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    # optical [right,down,forward] -> camera FLU -> base FLU
    R = (
        np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    )
    return R, np.array(CAMERA_POSITIONS[side]) + np.array([offset, 0.0, 0.0])


def unproject(depth, K, side, pitch=0.65, offset=0.0):
    depth = np.where(np.isfinite(depth), depth, np.nan)
    v, u = np.indices(depth.shape)
    xyz = np.stack(
        ((u - K[2]) * depth / K[0], (v - K[3]) * depth / K[1], depth), axis=-1
    )
    R, t = transform(side, pitch, offset)
    return xyz @ R.T + t


def clearance(x, y):
    return np.hypot(np.maximum(np.abs(x) - 1.3, 0), np.maximum(np.abs(y) - 0.9, 0))


def detect(rgb, depth, K, side):
    pts = unproject(depth, K, side)
    valid = (
        np.isfinite(depth)
        & (depth > 0.1)
        & (depth < 15)
        & (pts[:, :, 2] > 0.18)
        & (pts[:, :, 2] < 3.5)
    )
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    masks = {
        "person": ((hsv[:, :, 0] < 12) | (hsv[:, :, 0] > 170)),
        "obstacle": ((hsv[:, :, 0] > 100) & (hsv[:, :, 0] < 135)),
    }
    out = []
    for label, mask in masks.items():
        mask = (mask & (hsv[:, :, 1] > 120) & (hsv[:, :, 2] > 45) & valid).astype(
            "uint8"
        )
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        for j in range(1, count):
            x, y, w, h, area = stats[j]
            if area < 12:
                continue
            cloud = pts[labels == j]
            pos = np.median(cloud, axis=0)
            out.append(
                dict(
                    label=label,
                    bbox=[int(x), int(y), int(w), int(h)],
                    position=pos.tolist(),
                    distance=float(
                        np.percentile(clearance(cloud[:, 0], cloud[:, 1]), 10)
                    ),
                    side=side,
                )
            )
    return out, pts[valid][::8]


def level(distance, thresholds=(1.5, 3.0, 6.0)):
    return (
        3
        if distance <= thresholds[0]
        else 2 if distance <= thresholds[1] else 1 if distance <= thresholds[2] else 0
    )


class Controller:
    def __init__(self, limits=(1.5, 3.0, 6.0), release=0.4, decel=1.0):
        self.limits = limits
        self.release = release
        self.decel = decel
        self.state = 3
        self.v = 0.0
        self.brake_hold = limits[0]
        self.w = 0.0

    def step(
        self,
        distance,
        requested,
        dt,
        healthy=True,
        measured_speed=None,
        requested_yaw=0.0,
        measured_yaw=0.0,
        braking=False,
    ):
        speed = max(abs(self.v), abs(measured_speed or 0.0)) + math.hypot(
            1.3, 0.9
        ) * max(abs(self.w), abs(measured_yaw))
        self.stopping_distance = speed * 0.4 + speed * speed / (2 * self.decel)
        limits = (
            max(self.limits[0], self.stopping_distance + 0.5),
            max(self.limits[1], self.stopping_distance + 2),
            max(self.limits[2], self.stopping_distance + 5),
        )
        if self.state == 3:
            limits = (
                max(limits[0], self.brake_hold),
                max(limits[1], self.brake_hold + 1.5),
                max(limits[2], self.brake_hold + 4.5),
            )
        self.brake_threshold = limits[0]
        wanted = level(distance, limits) if healthy else 3
        if wanted >= self.state:
            self.state = wanted
        elif self.state and distance > limits[3 - self.state] + self.release:
            self.state = wanted
        if self.state == 3:
            self.brake_hold = limits[0]
        else:
            self.brake_hold = self.limits[0]
        target = float(
            np.clip(
                requested,
                -[MAX_SPEED, 1.5, 0.6, 0.0][self.state],
                [MAX_SPEED, 1.5, 0.6, 0.0][self.state],
            )
        )
        yaw_target = float(
            np.clip(
                requested_yaw,
                -[0.55, 0.35, 0.2, 0.0][self.state],
                [0.55, 0.35, 0.2, 0.0][self.state],
            )
        )
        if braking:
            target = 0.0
            yaw_target = 0.0
        deceleration = (
            self.decel if self.state == 3 or braking or requested * self.v < 0 else 0.25
        )
        self.v = slew_signed(self.v, target, dt, 0.4, deceleration)
        self.w = slew_signed(self.w, yaw_target, dt, 0.5, 1.2)
        return self.state, self.v


def slew_signed(current, target, dt, accel, decel):
    """Linear ramp in either direction; reverse requests first stop at zero."""
    if current * target < 0:
        target = 0.0
    rate = accel if abs(target) > abs(current) else decel
    return float(
        np.clip(target, current - rate * max(0.0, dt), current + rate * max(0.0, dt))
    )


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
