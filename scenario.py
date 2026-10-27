#!/usr/bin/env python3
"""24 seeded layouts, truth only for evaluation. Start after start.sh, vehicle stationary."""

import json, math, os, subprocess, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry

os.environ.setdefault("GZ_PARTITION", "excavator_safety")
os.environ.setdefault("ROS_DOMAIN_ID", "33")
rclpy.init()
n = Node("scenario_evaluator")
n.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
rng = np.random.default_rng(42)
state = {}
pose = [0.0, 0.0]
n.create_subscription(
    String, "/safety/status", lambda m: state.update(json.loads(m.data)), 10
)


def odom(m):
    pose[:] = [m.pose.pose.position.x, m.pose.pose.position.y]


n.create_subscription(Odometry, "/safety/odom", odom, 10)
speed = n.create_publisher(Float32, "/safety/requested_speed", 10)


def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)


def move(name, x, y):
    request = f'name: "{name}" position: {{x: {x} y: {y} z: 0}} orientation: {{w: 1}}'
    result = subprocess.run(
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
            "3000",
            "--req",
            request,
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode or "true" not in result.stdout:
        raise RuntimeError(result.stderr + result.stdout)


spin(2)
speed.publish(Float32(data=0.0))
spin(1)
P = Path(__file__).parent
f = open(P / "data/scenarios.jsonl", "w", buffering=1)
from scene_fixtures import hide_targets

hide_targets()
for trial in range(24):
    targets = []
    for j in range(8):
        # One visible target, random bearing/range and alternating semantic class.
        active = j == trial % 8
        angle = [0, math.pi / 2, math.pi, -math.pi / 2][trial % 4] + rng.uniform(
            -0.18, 0.18
        )
        radius = [2.8, 4.0, 6.0, 9.0][(trial // 4) % 4] + rng.uniform(-0.15, 0.15)
        x, y = (
            (radius * math.cos(angle) + pose[0], radius * math.sin(angle) + pose[1])
            if active
            else (40 + j, 40)
        )
        if active or (trial > 0 and j == (trial - 1) % 8):
            move(f"target_{j}", x, y)
        if active:
            targets.append(
                dict(
                    id=j,
                    label="person" if j % 2 == 0 else "obstacle",
                    x=x,
                    y=y,
                    half=0.25 if j % 2 == 0 else 0.4,
                )
            )
    exposure = n.get_clock().now().nanoseconds / 1e9
    spin(0.6)
    start = n.get_clock().now().nanoseconds / 1e9
    spin(2)
    f.write(
        json.dumps(
            dict(
                trial=trial,
                exposure_stamp=exposure,
                stamp_start=start,
                stamp_end=n.get_clock().now().nanoseconds / 1e9,
                targets=targets,
                vehicle_xy=pose.copy(),
                status=state.copy(),
            )
        )
        + "\n"
    )
    print(trial, state.get("state"), flush=True)
f.close()
n.destroy_node()
rclpy.shutdown()
