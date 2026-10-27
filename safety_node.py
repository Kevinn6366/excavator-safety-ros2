#!/usr/bin/env python3
import json, time, math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import String, Float32, Header
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from core import SIDES, detect, Controller, assess_path, level, MAX_SPEED
from ros_gz_interfaces.msg import Contacts


class Safety(Node):
    def __init__(self):
        super().__init__("excavator_safety")
        (
            self.declare_parameter("use_sim_time", True)
            if not self.has_parameter("use_sim_time")
            else None
        )
        for k, v in [
            ("danger", 0.25),
            ("warning", 0.8),
            ("notice", 2.0),
            ("sensor_timeout", 0.8),
            ("requested_speed", 0.0),
            ("distance_margin", 0.15),
        ]:
            self.declare_parameter(k, v)
        self.control = Controller(
            tuple(self.get_parameter(x).value for x in ["danger", "warning", "notice"])
        )
        self.cv = CvBridge()
        self.K = {}
        self.latest = {}
        self.sync = []
        self.seq = 0
        self.lastsave = {}
        self.speed = 0.0
        self.yaw_speed = 0.0
        self.odom_stamp = -100.0
        self.requested_yaw = 0.0
        self.motion_received = None
        self.driver_brake = False
        self.last_contact = -100.0
        self.collision_count = 0
        self.contact_names = []
        self.requested = self.get_parameter("requested_speed").value
        self.out = Path(__file__).parent / "data"
        self.out.mkdir(exist_ok=True)
        self.log = open(self.out / "events.jsonl", "a", buffering=1)
        self.cmd = self.create_publisher(Twist, "/safety/cmd_vel", 10)
        self.status = self.create_publisher(String, "/safety/status", 10)
        self.detections_pub = self.create_publisher(String, "/safety/detections", 10)
        self.scan_pub = self.create_publisher(String, "/safety/visual_scan", 10)
        self.cloud = self.create_publisher(PointCloud2, "/safety/points_base", 10)
        self.create_subscription(
            Float32, "/safety/requested_speed", self.legacy_request, 10
        )
        self.create_subscription(
            String, "/safety/requested_motion", self.motion_request, 10
        )
        self.create_subscription(Contacts, "/safety/contacts", self.contact, 10)
        self.create_subscription(
            Odometry, "/safety/odom", self.odometry, qos_profile_sensor_data
        )
        for side in SIDES:
            self.create_subscription(
                CameraInfo,
                f"/safety/{side}/camera_info",
                lambda m, s=side: self.K.update({s: [m.k[0], m.k[4], m.k[2], m.k[5]]}),
                qos_profile_sensor_data,
            )
            a = Subscriber(
                self,
                Image,
                f"/safety/{side}/image",
                qos_profile=qos_profile_sensor_data,
            )
            b = Subscriber(
                self,
                Image,
                f"/safety/{side}/depth_image",
                qos_profile=qos_profile_sensor_data,
            )
            sync = ApproximateTimeSynchronizer([a, b], 10, 0.04)
            sync.registerCallback(lambda rgb, depth, s=side: self.frame(s, rgb, depth))
            self.sync.append((a, b, sync))
        self.prev = self.get_clock().now().nanoseconds / 1e9
        self.create_timer(0.05, self.tick)

    def legacy_request(self, m):
        if not math.isfinite(m.data):
            return
        self.requested = float(np.clip(m.data, -MAX_SPEED, MAX_SPEED))
        self.requested_yaw = 0.0
        self.motion_received = None
        self.driver_brake = abs(m.data) < 0.001

    def motion_request(self, m):
        try:
            data = json.loads(m.data)
            v = data["speed"]
            w = data["yaw"]
            brake = data.get("brake", False)
            if (
                not all(
                    isinstance(x, (int, float))
                    and not isinstance(x, bool)
                    and math.isfinite(x)
                    for x in [v, w]
                )
                or type(brake) is not bool
            ):
                return
        except (ValueError, TypeError, KeyError):
            return
        self.requested = float(np.clip(v, -MAX_SPEED, MAX_SPEED))
        self.requested_yaw = float(np.clip(w, -0.55, 0.55))
        self.driver_brake = brake
        self.motion_received = time.monotonic()

    def contact(self, m):
        names = sorted(
            {
                name
                for c in m.contacts
                for name in [c.collision1.name, c.collision2.name]
                if "target_" in name
            }
        )
        if not names:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_contact > 0.5:
            self.collision_count += 1
            self.log.write(
                json.dumps(dict(kind="collision", stamp=now, targets=names)) + "\n"
            )
        self.last_contact = now
        self.contact_names = names

    def odometry(self, m):
        self.speed = m.twist.twist.linear.x
        self.yaw_speed = m.twist.twist.angular.z
        self.odom_stamp = m.header.stamp.sec + m.header.stamp.nanosec / 1e9

    def frame(self, side, rgb, dep):
        if side not in self.K:
            return
        start = time.perf_counter()
        image = self.cv.imgmsg_to_cv2(rgb, "rgb8")
        depth = self.cv.imgmsg_to_cv2(dep, "32FC1")
        targets, cloud = detect(image, depth, self.K[side], side)
        stamp = rgb.header.stamp.sec + rgb.header.stamp.nanosec / 1e9
        self.latest[side] = (
            stamp if np.isfinite(depth).sum() > 100 else -100.0,
            targets,
        )
        self.detections_pub.publish(
            String(
                data=json.dumps(
                    dict(
                        stamp=stamp,
                        side=side,
                        targets=targets,
                        source="rgb_depth_detection",
                        rgb_topic=f"/safety/{side}/image",
                        depth_topic=f"/safety/{side}/depth_image",
                    )
                )
            )
        )
        scan = cloud[np.isfinite(cloud).all(axis=1)]
        scan = scan[
            (np.linalg.norm(scan[:, :2], axis=1) < 11)
            & ((np.abs(scan[:, 0]) > 1.4) | (np.abs(scan[:, 1]) > 1.0))
        ]
        scan = scan[:: max(1, int(np.ceil(len(scan) / 180)))]
        self.scan_pub.publish(
            String(
                data=json.dumps(
                    dict(
                        stamp=stamp,
                        side=side,
                        source="depth_backprojection",
                        points=scan.round(3).tolist(),
                        depth_topic=f"/safety/{side}/depth_image",
                    )
                )
            )
        )
        header = Header()
        header.stamp = rgb.header.stamp
        header.frame_id = "base_link"
        self.cloud.publish(create_cloud_xyz32(header, cloud.tolist()))
        if stamp - self.lastsave.get(side, -100) >= 0.5:
            self.lastsave[side] = stamp
            self.seq += 1
            if self.seq <= 480:
                np.savez_compressed(
                    self.out / f"frame_{self.seq:04d}_{side}.npz",
                    rgb=image,
                    depth=depth,
                    K=self.K[side],
                    points=cloud,
                    stamp=stamp,
                    side=side,
                )
            self.log.write(
                json.dumps(
                    dict(
                        kind="detection",
                        stamp=stamp,
                        side=side,
                        targets=targets,
                        processing_ms=(time.perf_counter() - start) * 1000,
                    )
                )
                + "\n"
            )

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = max(0, min(now - self.prev, 0.1))
        self.prev = now
        healthy = (
            0 <= now - self.odom_stamp < 0.8
            and len(self.latest) == 4
            and all(
                0 <= now - t < self.get_parameter("sensor_timeout").value
                for t, _ in self.latest.values()
            )
        )
        targets = [
            x for t, items in self.latest.values() if 0 <= now - t < 0.8 for x in items
        ]
        d = min((x["distance"] for x in targets), default=99.0)
        input_timeout = (
            self.motion_received is not None
            and time.monotonic() - self.motion_received > 1.0
        )
        if input_timeout:
            self.requested = 0.0
            self.requested_yaw = 0.0
            self.driver_brake = True
        velocity = (
            self.speed if abs(self.speed) > abs(self.control.v) else self.control.v
        )
        omega = (
            self.yaw_speed
            if abs(self.yaw_speed) > abs(self.control.w)
            else self.control.w
        )
        risk = assess_path(targets, velocity, omega, self.requested, self.requested_yaw)
        decision_d = max(
            0.0, risk["distance"] - self.get_parameter("distance_margin").value
        )
        state, v = self.control.step(
            decision_d if risk["side"] is not None else float("inf"),
            self.requested,
            dt,
            healthy,
            self.speed,
            self.requested_yaw,
            self.yaw_speed,
            self.driver_brake,
        )
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = self.control.w
        self.cmd.publish(cmd)
        rec = dict(
            proximity_state=["SAFE", "NOTICE", "WARNING", "DANGER"][
                level(max(0.0, d - self.get_parameter("distance_margin").value))
            ],
            collision_active=now - self.last_contact < 0.25,
            collision_recent=now - self.last_contact < 5.0,
            collision_count=self.collision_count,
            collision_targets=self.contact_names,
            path_distance=risk["distance"],
            ttc=risk["ttc"],
            closing_speed=risk["closing_speed"],
            risk_side=risk["side"],
            driver_brake=self.driver_brake,
            input_timeout=input_timeout,
            requested_yaw=self.requested_yaw,
            command_yaw=self.control.w,
            measured_yaw=self.yaw_speed,
            aeb_active=bool(
                state == 3
                and healthy
                and (abs(self.requested) > 0 or abs(self.requested_yaw) > 0)
            ),
            brake_reason=(
                "sensor_unavailable"
                if not healthy
                else "obstacle" if state == 3 else None
            ),
            brake_threshold=self.control.brake_threshold,
            stopping_distance=self.control.stopping_distance,
            brake_time=max(
                abs(self.speed) / self.control.decel, abs(self.yaw_speed) / 1.2
            ),
            kind="control",
            stamp=now,
            state=["SAFE", "NOTICE", "WARNING", "DANGER"][state],
            distance=d,
            decision_distance=decision_d,
            distance_margin=self.get_parameter("distance_margin").value,
            healthy=healthy,
            requested=self.requested,
            command=v,
            measured_speed=self.speed,
            light=["green", "yellow", "orange", "red"][state],
            buzzer=state > 0,
            brake=state == 3,
        )
        self.status.publish(String(data=json.dumps(rec)))
        self.log.write(json.dumps(rec) + "\n")


def main():
    rclpy.init()
    n = Safety()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            n.cmd.publish(Twist())
        n.log.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
