"""Shared sensor mounting and vehicle configuration (SI units)."""

import math

MAX_SPEED = 50.0 / 3.6
SIDES = {"front": 0.0, "left": math.pi / 2, "rear": math.pi, "right": -math.pi / 2}
CAMERA_POSITIONS = {
    "front": (1.5, 0.55, 2.8),
    "left": (0.0, 1.0, 2.8),
    "rear": (-1.2, 0.0, 2.8),
    "right": (0.0, -1.0, 2.8),
}
