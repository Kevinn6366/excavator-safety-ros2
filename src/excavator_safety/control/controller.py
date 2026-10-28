"""Risk levels, hysteresis and signed acceleration/braking ramps."""

import math
import numpy as np
from excavator_safety.config import MAX_SPEED


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
