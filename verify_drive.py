"""Physical Gazebo velocity feedback test. Run after scenario.py while start.sh is active."""

import os, json, time, subprocess
from pathlib import Path

os.environ.setdefault("ROS_DOMAIN_ID", "33")
os.environ.setdefault("GZ_PARTITION", "excavator_safety")
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry

rclpy.init()
n = Node("drive_acceptance")
state = {}
pose = [0.0, 0.0]
samples = []


def status(m):
    state.update(json.loads(m.data))
    samples.append(state.copy())


def odom(m):
    pose[:] = [m.pose.pose.position.x, m.pose.pose.position.y]


n.create_subscription(String, "/safety/status", status, 10)
n.create_subscription(Odometry, "/safety/odom", odom, 10)
pub = n.create_publisher(Float32, "/safety/requested_speed", 10)


def spin(t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)


def move(j, x, y):
    p = subprocess.run(
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
            f'name: "target_{j}" position: {{x: {x} y: {y} z: 0}} orientation: {{w: 1}}',
        ],
        capture_output=True,
        text=True,
    )
    if "true" not in p.stdout:
        spin(0.5)
        p = subprocess.run(
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
                "8000",
                "--req",
                f'name: "target_{j}" position: {{x: {x} y: {y} z: 0}} orientation: {{w: 1}}',
            ],
            capture_output=True,
            text=True,
        )
        if "true" not in p.stdout:
            raise RuntimeError(p.stderr + p.stdout)


try:
    spin(1)
    from scene_fixtures import hide_targets

    hide_targets()
    pub.publish(Float32(data=1.0))
    spin(10)
    peak = max((x["measured_speed"] for x in samples), default=0)
    before = state.copy()
    move(0, pose[0] + 2.7, pose[1])
    trigger = state.get("stamp", 0)
    spin(8)
    after = [x for x in samples if x["stamp"] >= trigger]
    brake = next((x for x in after if x["brake"]), None)
    stopped = next(
        (x for x in after if x["brake"] and abs(x["measured_speed"]) < 0.05), None
    )
    result = dict(
        peak_measured_speed=peak,
        brake_triggered=brake is not None,
        stopped=stopped is not None,
        brake_delay_sim_s=brake["stamp"] - trigger if brake else None,
        stop_delay_sim_s=stopped["stamp"] - trigger if stopped else None,
        before=before,
        final=state,
        passed=peak > 0.5 and brake is not None and stopped is not None,
    )
    (Path(__file__).parent / "data/drive_test.json").write_text(
        json.dumps(result, indent=2)
    )
    print(json.dumps(result, indent=2))
finally:
    pub.publish(Float32(data=0.0))
    spin(0.2)
    n.destroy_node()
    rclpy.shutdown()
