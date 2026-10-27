"""Gazebo test fixtures only. Never imported by perception or control nodes."""

import subprocess


def hide_targets():
    req = " ".join(
        f'pose: {{name: "target_{j}" position: {{x: {-350-j*.3} y: 80 z: 0}} orientation: {{w: 1}}}}'
        for j in range(80)
    )
    r = subprocess.run(
        [
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
            req,
        ],
        capture_output=True,
        text=True,
        timeout=8,
    )
    if r.returncode or "true" not in r.stdout:
        raise RuntimeError(r.stderr + r.stdout)
