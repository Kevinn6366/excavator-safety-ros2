#!/usr/bin/env python3
"""Local driving-console HTTP monitor for real ROS camera topics; no Flask dependency."""

import json, threading, time, os, secrets, math
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs
import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sim_manager import SimulationManager
from core import MAX_SPEED
from birdseye import Birdseye
from cv_bridge import CvBridge

SIDES = ("front", "rear", "left", "right")
P = Path(__file__).parent


class Monitor(Node):
    def __init__(self):
        super().__init__("excavator_web_monitor")
        self.lock = threading.Lock()
        self.cv = CvBridge()
        self.frames = {}
        self.received = {}
        self.stamps = {}
        self.sequence = {}
        self.times = {}
        self.status = None
        self.status_time = 0
        self.detections = {}
        self.last_encode = {}
        self.pose = None
        self.travel = 0.0
        self.odom_stamp = None
        self.raw_rgb = {}
        self.raw_pairs = {}
        self.intrinsics = {}
        self.det_received = {}
        self.scans = {}
        self.scan_received = {}
        self.pano = Birdseye()
        self.pano_frame = None
        self.pano_info = {}
        self.pano_time = 0
        self.pano_seq = 0
        self.stop_pub = self.create_publisher(Float32, "/safety/requested_speed", 10)
        self.motion_pub = self.create_publisher(String, "/safety/requested_motion", 10)
        self.create_subscription(
            Odometry, "/safety/odom", self.odometry, qos_profile_sensor_data
        )
        self.create_subscription(String, "/safety/status", self.control, 10)
        self.create_subscription(String, "/safety/detections", self.detection, 10)
        self.create_subscription(String, "/safety/visual_scan", self.scan, 10)
        self.create_timer(0.7, self.build_panorama)
        for side in SIDES:
            self.create_subscription(
                CameraInfo,
                f"/safety/{side}/camera_info",
                lambda m, s=side: self.intrinsics.update(
                    {s: [m.k[0], m.k[4], m.k[2], m.k[5]]}
                ),
                qos_profile_sensor_data,
            )
            for mode, suffix in [("rgb", "image"), ("depth", "depth_image")]:
                self.create_subscription(
                    Image,
                    f"/safety/{side}/{suffix}",
                    lambda m, s=side, k=mode: self.frame(s, k, m),
                    qos_profile_sensor_data,
                )

    def odometry(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        pose = [
            m.pose.pose.position.x,
            m.pose.pose.position.y,
            m.pose.pose.position.z,
            yaw,
        ]
        stamp = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
        with self.lock:
            if self.odom_stamp is not None and stamp < self.odom_stamp:
                self.travel = 0.0
                self.pose = None
            if self.pose is not None:
                self.travel += math.hypot(
                    pose[0] - self.pose[0], pose[1] - self.pose[1]
                )
            self.pose = pose
            self.odom_stamp = stamp

    def request_stop(self):
        self.stop_pub.publish(Float32(data=0.0))

    def control(self, m):
        try:
            x = json.loads(m.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.status = x
            self.status_time = time.monotonic()

    def detection(self, m):
        try:
            x = json.loads(m.data)
        except (ValueError, TypeError):
            return
        if x.get("side") in SIDES:
            with self.lock:
                self.detections[x["side"]] = x
                self.det_received[x["side"]] = time.monotonic()

    def scan(self, m):
        try:
            x = json.loads(m.data)
        except (ValueError, TypeError):
            return
        if x.get("side") in SIDES and x.get("source") == "depth_backprojection":
            with self.lock:
                self.scans[x["side"]] = x
                self.scan_received[x["side"]] = time.monotonic()

    def build_panorama(self):
        now = time.monotonic()
        with self.lock:
            frames = {}
            depths = {}
            stamps = []
            for side in SIDES:
                if side not in self.intrinsics:
                    return
                rgb = self.raw_pairs.get((side, "rgb"), [])
                dep = self.raw_pairs.get((side, "depth"), [])
                pairs = [
                    (r, d)
                    for r in rgb
                    for d in dep
                    if abs(r[0] - d[0]) < 0.04 and now - r[2] < 1.2 and now - d[2] < 1.2
                ]
                if not pairs:
                    return
                r, d = max(pairs, key=lambda pair: pair[0][0])
                frames[side] = r[1]
                depths[side] = d[1]
                stamps.extend([r[0], d[0]])
            if max(stamps) - min(stamps) > 0.25:
                return
            K = dict(self.intrinsics)
        output, coverage = self.pano.stitch(frames, K, depths)
        ok, jpg = cv2.imencode(".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if ok:
            with self.lock:
                self.pano_frame = jpg.tobytes()
                self.pano_time = now
                self.pano_seq += 1
                self.pano_info = dict(
                    stamp=min(stamps),
                    skew=max(stamps) - min(stamps),
                    coverage=coverage,
                    source="four_rgbd_orthographic",
                    width=600,
                    height=600,
                    extent=7,
                    camera_pitch=0.65,
                )

    def frame(self, side, mode, msg):
        key = (side, mode)
        now = time.monotonic()
        try:
            raw = self.cv.imgmsg_to_cv2(msg, "bgr8" if mode == "rgb" else "32FC1")
            with self.lock:
                self.raw_pairs.setdefault(key, deque(maxlen=6)).append(
                    (
                        msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                        raw.copy(),
                        now,
                    )
                )
            if now - self.last_encode.get(key, 0) < 0.16:
                return
            self.last_encode[key] = now
            if mode == "rgb":
                bgr = raw
            else:
                depth = raw
                valid = np.isfinite(depth) & (depth > 0.1) & (depth < 15)
                gray = (
                    np.clip(np.nan_to_num(depth, nan=15, posinf=15, neginf=15), 0, 15)
                    / 15
                    * 255
                ).astype(np.uint8)
                bgr = cv2.applyColorMap(255 - gray, cv2.COLORMAP_TURBO)
                bgr[~valid] = 0
            ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                return
        except Exception as exc:
            self.get_logger().warning(str(exc))
            return
        with self.lock:
            if mode == "rgb":
                self.raw_rgb[side] = bgr.copy()
            self.frames[key] = encoded.tobytes()
            self.received[key] = now
            self.stamps[key] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            self.sequence[key] = self.sequence.get(key, 0) + 1
            history = self.times.setdefault(key, deque(maxlen=15))
            history.append(now)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            cameras = {}
            for side in SIDES:
                cameras[side] = {}
                for mode in ["rgb", "depth"]:
                    key = (side, mode)
                    h = self.times.get(key, [])
                    age = now - self.received[key] if key in self.received else None
                    cameras[side][mode] = dict(
                        age=age,
                        online=age is not None and age < 2,
                        stamp=self.stamps.get(key),
                        seq=self.sequence.get(key, 0),
                        fps=(
                            (len(h) - 1) / (h[-1] - h[0])
                            if len(h) > 1 and h[-1] > h[0] and age < 2
                            else 0
                        ),
                    )
            return dict(
                travel_m=self.travel,
                status=self.status,
                status_age=now - self.status_time if self.status else None,
                cameras=cameras,
                detections={
                    s: {**v, "age": now - self.det_received[s]}
                    for s, v in self.detections.items()
                    if now - self.det_received[s] < 1.2
                },
                visual_scan={
                    s: {**v, "age": now - self.scan_received[s]}
                    for s, v in self.scans.items()
                    if now - self.scan_received[s] < 1.2
                },
                panorama={
                    **self.pano_info,
                    "seq": self.pano_seq,
                    "age": now - self.pano_time if self.pano_time else None,
                    "online": self.pano_time > 0
                    and now - self.pano_time < 2
                    and all(
                        cameras[s][m]["online"] for s in SIDES for m in ["rgb", "depth"]
                    ),
                },
            )


def main():
    rclpy.init()
    node = Monitor()
    manager = SimulationManager(node)
    token = secrets.token_urlsafe(32)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == "/api/state":
                data = node.snapshot()
                data["simulation"] = manager.snapshot()
                data["control_token"] = token
                body = json.dumps(data, allow_nan=False).encode()
                mime = "application/json; charset=utf-8"
            elif parsed.path in ("/frame/panorama.jpg", "/frame/birdseye.jpg"):
                with node.lock:
                    body = node.pano_frame
                if body is None:
                    self.send_error(503, "Waiting for synchronized camera images")
                    return
                mime = "image/jpeg"
            elif parsed.path.startswith("/frame/") and parsed.path.endswith(".jpg"):
                side = parsed.path.split("/")[-1][:-4]
                mode = parse_qs(parsed.query).get("view", ["rgb"])[0]
                if side not in SIDES or mode not in ["rgb", "depth"]:
                    self.send_error(404)
                    return
                with node.lock:
                    body = node.frames.get((side, mode))
                if body is None:
                    self.send_error(503, "Waiting for ROS image")
                    return
                mime = "image/jpeg"
            elif parsed.path == "/zx200-roof.png":
                body = (P / "web/dist/zx200-roof.png").read_bytes()
                mime = "image/png"
            elif parsed.path in ("/", "/index.html"):
                body = (P / "web/dist/index.html").read_bytes()
                mime = "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            if self.path not in ("/api/action", "/api/drive", "/api/teleop"):
                self.send_error(404)
                return
            origin = self.headers.get("Origin")
            if (
                origin and urlparse(origin).netloc != self.headers.get("Host")
            ) or not secrets.compare_digest(
                self.headers.get("X-Local-Control", ""), token
            ):
                self.send_error(403)
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 1024:
                    raise ValueError("请求大小无效")
                payload = json.loads(self.rfile.read(size))
                if self.path in ("/api/drive", "/api/teleop"):
                    speed = payload.get("speed", 0.0)
                    yaw = payload.get("yaw", 0.0)
                    brake = payload.get("brake", False)
                    if type(brake) is not bool:
                        raise ValueError("brake须为布尔值")
                    if (
                        isinstance(speed, bool)
                        or not isinstance(speed, (float, int))
                        or not math.isfinite(speed)
                        or not -MAX_SPEED <= speed <= MAX_SPEED
                    ):
                        raise ValueError("目标速度须在±50 km/h以内")
                    if (
                        isinstance(yaw, bool)
                        or not isinstance(yaw, (float, int))
                        or not math.isfinite(yaw)
                        or not -0.55 <= yaw <= 0.55
                    ):
                        raise ValueError("转向角速度须为−0.55–0.55 rad/s")
                    state = node.snapshot()
                    sim = manager.snapshot()
                    if (abs(speed) > 0 or abs(yaw) > 0) and (
                        sim["job"]["state"] == "running"
                        or not sim["running"]
                        or state["status_age"] is None
                        or state["status_age"] > 1
                        or not state["status"]["healthy"]
                    ):
                        raise RuntimeError("请等待仿真和传感器就绪")
                    if self.path == "/api/teleop":
                        node.motion_pub.publish(
                            String(
                                data=json.dumps(
                                    dict(
                                        speed=float(speed), yaw=float(yaw), brake=brake
                                    )
                                )
                            )
                        )
                    else:
                        node.stop_pub.publish(Float32(data=float(speed)))
                    result = {
                        "requested": speed,
                        "requested_yaw": yaw if self.path == "/api/teleop" else 0,
                    }
                    code = 200
                else:
                    result = manager.submit(
                        payload.get("action"),
                        payload.get("people", 24),
                        payload.get("obstacles", 24),
                    )
                    code = 202
            except (ValueError, TypeError, AttributeError) as exc:
                result = {"error": str(exc)}
                code = 400
            except RuntimeError as exc:
                result = {"error": str(exc)}
                code = 409
            body = json.dumps(result).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(
        (
            os.environ.get("MONITOR_HOST", "0.0.0.0"),
            int(os.environ.get("MONITOR_PORT", "8080")),
        ),
        Handler,
    )
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("Camera monitor listening on http://0.0.0.0:8080", flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
