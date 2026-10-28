"""HSV connected-component detection with depth filtering."""

import cv2
import numpy as np
from .geometry import unproject, clearance


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
