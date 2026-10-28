"""Put visible demonstration targets around the stopped vehicle (no driving command)."""

import os, math, subprocess, time

os.environ.setdefault("ROS_DOMAIN_ID", "33")
os.environ.setdefault("GZ_PARTITION", "excavator_safety")
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

rclpy.init()
n = Node("demo_layout")
pose = []
n.create_subscription(
    Odometry,
    "/safety/odom",
    lambda m: pose.__setitem__(
        slice(None), [m.pose.pose.position.x, m.pose.pose.position.y]
    ),
    10,
)
pub = n.create_publisher(Float32, "/safety/requested_speed", 10)
end = time.monotonic() + 5
while not pose and time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
if not pose:
    raise RuntimeError("No vehicle odometry")
pub.publish(Float32(data=0.0))
for j, (angle, radius) in enumerate(
    [
        (0, 4.5),
        (math.pi / 2, 5.5),
        (math.pi, 5),
        (-math.pi / 2, 6),
        (math.pi / 4, 7),
        (3 * math.pi / 4, 8),
        (-3 * math.pi / 4, 7),
        (-math.pi / 4, 8),
    ]
):
    x, y = pose[0] + radius * math.cos(angle), pose[1] + radius * math.sin(angle)
    cmd = [
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
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if "true" not in result.stdout:
        raise RuntimeError(result.stderr + result.stdout)
print("Eight demonstration targets placed around stopped vehicle", pose)
n.destroy_node()
rclpy.shutdown()
