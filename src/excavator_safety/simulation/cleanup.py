"""Terminate only this project's simulator children, including detached gz servers.
Both exact Gazebo partition and working directory must match; dashboard and other
coursework processes are excluded by executable/argument checks.
"""

from excavator_safety.paths import ROOT

import os, signal, time
from pathlib import Path

P = ROOT


def owned(pid):
    try:
        root = Path("/proc") / str(pid)
        if root.joinpath("cwd").resolve() != P:
            return False
        env = root.joinpath("environ").read_bytes().split(b"\0")
        if b"GZ_PARTITION=excavator_safety" not in env:
            return False
        args = root.joinpath("cmdline").read_bytes().split(b"\0")
        cmd = b" ".join(args)
        exe = Path(os.fsdecode(args[0])).name
        return (
            cmd.startswith(b"gz sim ")
            or (exe in {"ruby", "gz"} and b" sim " in cmd)
            or b"/ros_gz_bridge/parameter_bridge" in args[0]
            or (
                exe.startswith("python")
                and len(args) > 2
                and args[1] == b"-m"
                and args[2] == b"excavator_safety.ros.safety_node"
            )
        )
    except (OSError, ValueError):
        return False


def active():
    return [
        int(p.name)
        for p in Path("/proc").iterdir()
        if p.name.isdigit() and owned(int(p.name))
    ]


def send(pid, sig):
    if owned(pid):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass


if __name__ == "__main__":
    for pid in active():
        send(pid, signal.SIGTERM)
    end = time.monotonic() + 1.5
    while active() and time.monotonic() < end:
        time.sleep(0.1)
    for pid in active():
        send(pid, signal.SIGKILL)
