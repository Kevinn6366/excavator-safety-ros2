"""Project-scoped simulation lifecycle and reproducible scenario controls."""

from excavator_safety.paths import ROOT

import json, math, os, random, secrets, signal, subprocess, threading, time, fcntl
from pathlib import Path

P = ROOT


class SimulationManager:
    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.job = {"state": "idle", "action": None, "message": "准备就绪"}
        self.layout = None
        self.child = None

    def pid(self):
        try:
            pid = int((P / "run.pid").read_text())
            cwd = Path(f"/proc/{pid}/cwd").resolve()
            cmd = Path(f"/proc/{pid}/cmdline").read_bytes()
            return pid if cwd == P and b"start.sh" in cmd else None
        except (OSError, ValueError):
            return None

    def lock_free(self):
        with (P / ".run.lock").open("a") as f:
            try:
                fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return False
            fcntl.flock(f, fcntl.LOCK_UN)
            return True

    def snapshot(self):
        with self.lock:
            return dict(
                running=self.pid() is not None,
                job=dict(self.job),
                layout=(
                    {k: self.layout[k] for k in ["seed", "people", "obstacles", "time"]}
                    if self.layout
                    else None
                ),
            )

    def submit(self, action, people=24, obstacles=24):
        if action not in {"start", "stop", "reset", "randomize"}:
            raise ValueError("未知操作")
        if any(type(x) is not int or not 0 <= x <= 40 for x in [people, obstacles]):
            raise ValueError("人员和障碍物数量必须为0–40的整数")
        with self.lock:
            if self.job["state"] == "running":
                raise RuntimeError("上一个操作尚未完成")
            self.job = dict(
                state="running",
                action=action,
                message={
                    "start": "正在启动仿真",
                    "stop": "正在停止仿真",
                    "reset": "正在恢复初始场景",
                    "randomize": "正在布置随机场景",
                }[action],
                started=time.time(),
            )
        threading.Thread(
            target=self.execute, args=(action, people, obstacles), daemon=True
        ).start()
        return self.snapshot()

    def ready(self, timeout=55):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            s = self.node.snapshot()
            if (
                self.pid()
                and s["status_age"] is not None
                and s["status_age"] < 2
                and s["status"]["healthy"]
                and self.node.pose is not None
            ):
                return
            time.sleep(0.2)
        raise RuntimeError("仿真未在55秒内就绪，请检查logs/gazebo.log与logs/safety.log")

    def clear(self):
        with self.node.lock:
            self.node.status = None
            self.node.status_time = 0
            self.node.frames.clear()
            self.node.received.clear()
            self.node.detections.clear()
            self.node.stamps.clear()
            self.node.times.clear()
            self.node.raw_rgb.clear()
            self.node.raw_pairs.clear()
            self.node.scans.clear()
            self.node.scan_received.clear()
            self.node.det_received.clear()
            self.node.pano_frame = None
            self.node.pano_time = 0
            self.node.pano_info = {}
            self.node.pose = None
            self.node.travel = 0.0
            self.node.odom_stamp = None

    def stop(self):
        self.node.request_stop()
        subprocess.run([str(P / "scripts/stop.sh")], cwd=P, check=True, timeout=5)
        end = time.monotonic() + 12
        while (self.pid() or not self.lock_free()) and time.monotonic() < end:
            time.sleep(0.1)
        if self.pid() or not self.lock_free():
            raise RuntimeError("仿真进程尚未退出，未启动重复实例")
        self.clear()

    def start(self):
        if not self.pid():
            self.clear()
            (P / "logs").mkdir(exist_ok=True)
            with (P / "logs/dashboard_launch.log").open("a") as log:
                self.child = subprocess.Popen(
                    [str(P / "scripts/start.sh"), "--gui"],
                    cwd=P,
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=log,
                    start_new_session=True,
                )
        self.ready()

    def randomize(self, people, obstacles):
        self.ready(5)
        self.node.request_stop()
        end = time.monotonic() + 6
        while time.monotonic() < end:
            s = self.node.snapshot()["status"]
            if (
                s
                and abs(s["measured_speed"]) < 0.03
                and abs(s["command"]) < 0.01
                and abs(s.get("measured_yaw", 0)) < 0.02
                and abs(s.get("command_yaw", 0)) < 0.01
            ):
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("车辆未停止，未改动场景")
        with self.node.lock:
            pose = self.node.pose
        if pose is None:
            raise RuntimeError("尚未收到车辆位姿")
        seed = secrets.randbits(32)
        rng = random.Random(seed)
        placed = []
        records = []
        requests = []
        yaw = pose[3] if len(pose) > 3 else 0.0
        c, sn = math.cos(yaw), math.sin(yaw)
        available = [280.0]
        for origin, unit, bound in [(pose[0], c, 370), (pose[1], sn, 70)]:
            if abs(unit) > 1e-6:
                available.append((math.copysign(bound, unit) - origin) / unit)
        near = 25.0
        far = min(available)
        if far - near < 15 and people + obstacles:
            raise RuntimeError("当前朝向接近场地边界，请转向开阔区域或恢复场景")
        for j in range(80):
            active = (j // 2) < (people if j % 2 == 0 else obstacles)
            if active:
                for _ in range(1000):
                    forward = rng.uniform(near, far)
                    left = (-1 if j % 2 else 1) * rng.uniform(10, 20)
                    # Two isolated crossing events after a long clear approach. All other
                    # targets occupy the shoulders, so adding density does not block departure.
                    if j == 0:
                        forward = min(rng.uniform(100, 140), far * 0.5)
                        left = rng.uniform(-0.5, 0.5)
                    if j == 1:
                        forward = min(rng.uniform(200, 240), far * 0.9)
                        left = rng.uniform(-0.5, 0.5)
                    if j in [2, 3]:
                        forward = rng.uniform(-4, 4)
                        left = (-1 if j % 2 else 1) * rng.uniform(4.5, 6.0)
                    x = pose[0] + forward * c - left * sn
                    y = pose[1] + forward * sn + left * c
                    if all(math.hypot(x - u, y - v) > 1.6 for u, v in placed):
                        break
                else:
                    raise RuntimeError("未找到互不重叠的位置")
                placed.append((x, y))
            else:
                x, y = -350 - j * 0.3, 80
            requests.append(
                f'pose: {{name: "target_{j}" position: {{x: {x} y: {y} z: 0}} orientation: {{w: 1}}}}'
            )
            records.append(
                dict(
                    name=f"target_{j}",
                    active=active,
                    label="person" if j % 2 == 0 else "obstacle",
                    x=x,
                    y=y,
                )
            )
        args = [
            "gz",
            "service",
            "-s",
            "/world/safety/set_pose_vector",
            "--reqtype",
            "gz.msgs.Pose_V",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            " ".join(requests),
        ]
        result = subprocess.run(args, capture_output=True, text=True, timeout=8)
        if result.returncode or "true" not in result.stdout:
            raise RuntimeError("批量场景布置失败，请恢复场景后重试")
        layout = dict(
            seed=seed,
            people=people,
            obstacles=obstacles,
            targets=records,
            vehicle_xy=pose[:2],
            vehicle_yaw=yaw,
            time=time.time(),
        )
        with self.lock:
            self.layout = layout
        with (P / "data/web_scenarios.jsonl").open("a") as f:
            f.write(json.dumps(layout) + "\n")

    def execute(self, action, people, obstacles):
        try:
            if action == "stop":
                self.stop()
                message = "仿真已停止，控制台保持在线"
            elif action == "start":
                self.start()
                message = "仿真已就绪"
            elif action == "reset":
                self.stop()
                self.start()
                with self.lock:
                    self.layout = None
                message = "车辆、目标、仿真时间和速度已恢复初始状态"
            else:
                self.randomize(people, obstacles)
                message = f"已随机布置 {people} 名人员与 {obstacles} 个障碍物"
            with self.lock:
                self.job.update(state="success", message=message, finished=time.time())
        except Exception as exc:
            with self.lock:
                self.job.update(state="error", message=str(exc), finished=time.time())
