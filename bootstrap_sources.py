#!/usr/bin/env python3
"""Download the pinned ZX200 model and record verified source provenance."""

import hashlib
import json
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
COMMIT = "1b401d7cc97e11ad6a1c6807562b99c756366e4e"
REPOSITORY = "https://github.com/pwri-opera/OperaSim-AGX"
PREFIX = "Assets/Machines/Excavator/zx200/"
BASE = f"https://raw.githubusercontent.com/pwri-opera/OperaSim-AGX/{COMMIT}/"


def main():
    target = ROOT / "vendor/zx200"
    manifest = target / "SOURCE.json"
    if manifest.exists():
        data = json.loads(manifest.read_text())
        if data["commit"] != COMMIT:
            raise ValueError("Existing model uses a different revision")
        for record in data["files"]:
            path = target / record["path"]
            if hashlib.sha256(path.read_bytes()).hexdigest() != record["sha256"]:
                raise ValueError(f"Model file checksum mismatch: {path}")
        print("Pinned model files verified")
        return
    target.mkdir(parents=True, exist_ok=True)
    files = []

    def download(name, remote):
        path = target / name
        path.parent.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(BASE + remote, timeout=90) as response:
            content = response.read()
        temporary = path.with_suffix(path.suffix + ".part")
        temporary.write_bytes(content)
        temporary.replace(path)
        files.append({"path": name, "sha256": hashlib.sha256(content).hexdigest()})

    download("LICENSE", "LICENSE")
    download("zx200.urdf", PREFIX + "zx200.urdf")
    names = {
        x.get("filename").replace("package://", "")
        for x in ET.parse(target / "zx200.urdf").iter("mesh")
    }
    for name in sorted(names | {"meshes/bake.png"}):
        if Path(name).is_absolute() or ".." in Path(name).parts:
            raise ValueError(f"Unexpected asset path: {name}")
        download(name, PREFIX + name)
    manifest.write_text(
        json.dumps(
            {
                "repository": REPOSITORY,
                "commit": COMMIT,
                "source_directory": PREFIX,
                "files": files,
            },
            indent=2,
        )
    )
    print(f"Downloaded {len(files)} files to {target}")


if __name__ == "__main__":
    main()
