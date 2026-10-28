"""Regression checks for checkout paths and module-based process ownership."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from excavator_safety.paths import ROOT
from excavator_safety.simulation import cleanup


class LayoutTests(unittest.TestCase):
    def test_paths_are_independent_of_working_directory(self):
        env = dict(os.environ)
        env.pop("EXCAVATOR_SAFETY_ROOT", None)
        env["PYTHONPATH"] = str(ROOT / "src") + os.pathsep + env.get("PYTHONPATH", "")
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.check_output(
                [
                    sys.executable,
                    "-c",
                    "from excavator_safety.paths import ROOT, FRONTEND; "
                    'assert (FRONTEND / "index.html").is_file(); print(ROOT)',
                ],
                cwd=directory,
                env=env,
                text=True,
            )
        self.assertEqual(Path(result.strip()), ROOT)

    def test_explicit_root_override(self):
        with tempfile.TemporaryDirectory() as directory:
            env = dict(os.environ, EXCAVATOR_SAFETY_ROOT=directory)
            env["PYTHONPATH"] = (
                str(ROOT / "src") + os.pathsep + env.get("PYTHONPATH", "")
            )
            result = subprocess.check_output(
                [
                    sys.executable,
                    "-c",
                    "from excavator_safety.paths import ROOT; print(ROOT)",
                ],
                env=env,
                text=True,
            )
            self.assertEqual(Path(result.strip()), Path(directory).resolve())

    def test_only_owned_safety_module_is_selected_for_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            proc = Path(directory)
            process = proc / "123"
            process.mkdir()
            (process / "cwd").symlink_to(ROOT, target_is_directory=True)
            (process / "environ").write_bytes(b"GZ_PARTITION=excavator_safety\0")

            def mapped_path(value):
                return proc if value == "/proc" else Path(value)

            with patch.object(cleanup, "Path", side_effect=mapped_path):
                (process / "cmdline").write_bytes(
                    b"python3\0-m\0excavator_safety.ros.safety_node\0"
                )
                self.assertTrue(cleanup.owned(123))
                (process / "cmdline").write_bytes(
                    b"python3\0-m\0excavator_safety.ros.web_monitor\0"
                )
                self.assertFalse(cleanup.owned(123))
                (process / "cmdline").write_bytes(
                    b"python3\0-m\0excavator_safety.ros.safety_node\0"
                )
                (process / "environ").write_bytes(b"GZ_PARTITION=another_project\0")
                self.assertFalse(cleanup.owned(123))
