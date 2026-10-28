"""Camera projection and vehicle-frame geometry."""

import math
import numpy as np
from excavator_safety.config import SIDES, CAMERA_POSITIONS


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
