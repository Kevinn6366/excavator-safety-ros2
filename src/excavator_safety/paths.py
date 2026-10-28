"""Resolve checkout assets independently of the caller's working directory."""

import os
from pathlib import Path

ROOT = Path(
    os.environ.get("EXCAVATOR_SAFETY_ROOT", Path(__file__).resolve().parents[2])
).resolve()
FRONTEND = ROOT / "frontend"
DATA = ROOT / "data"
