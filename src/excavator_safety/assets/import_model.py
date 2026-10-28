"""Adapt Apache-2.0 OperaSim ZX200 meshes to a fixed compact excavator pose.
Original CAD meshes/materials are retained; no hydraulic physics is claimed.
"""

from excavator_safety.paths import ROOT

from pathlib import Path
import json, xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation

P = ROOT
source = P / "vendor/zx200/zx200.urdf"
r = E.parse(source).getroot()
scale = 0.4
poses = {"link1": np.eye(4)}
angles = {"joint1": 0.0, "joint2": -0.30, "joint3": 2.50, "joint4": -0.50}


def matrix(origin):
    T = np.eye(4)
    if origin is not None:
        T[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        T[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
    return T


pending = list(r.findall("joint"))
while pending:
    before = len(pending)
    for j in pending[:]:
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        if parent not in poses:
            continue
        mimic = j.find("mimic")
        q = angles.get(j.get("name"), 0.0)
        if mimic is not None:
            q = angles.get(mimic.get("joint"), 0) * float(
                mimic.get("multiplier", "1")
            ) + float(mimic.get("offset", "0"))
        axis = (
            np.fromstring(j.find("axis").get("xyz"), sep=" ")
            if j.find("axis") is not None
            else np.array([0, 0, 1.0])
        )
        A = np.eye(4)
        if j.get("type") == "prismatic":
            A[:3, 3] = axis * q
        elif j.get("type") != "fixed":
            A[:3, :3] = Rotation.from_rotvec(axis * q).as_matrix()
        poses[child] = poses[parent] @ matrix(j.find("origin")) @ A
        pending.remove(j)
    if before == len(pending):
        raise ValueError("Unresolved URDF joint tree")
out = E.Element("visuals")
for link in r.findall("link"):
    for idx, v in enumerate(link.findall("visual")):
        mesh = v.find("geometry/mesh")
        if mesh is None:
            continue
        path = P / "vendor/zx200" / mesh.get("filename").replace("package://", "")
        if not path.exists():
            raise FileNotFoundError(path)
        T = poses[link.get("name")] @ matrix(v.find("origin"))
        xyz = T[:3, 3] * scale + np.array([-0.12, 0.02, 0.1])
        rpy = Rotation.from_matrix(T[:3, :3]).as_euler("xyz")
        vis = E.SubElement(
            out, "visual", name="zx200_" + link.get("name") + "_" + str(idx)
        )
        E.SubElement(vis, "pose").text = " ".join(map(str, [*xyz, *rpy]))
        g = E.SubElement(vis, "geometry")
        new = E.SubElement(g, "mesh")
        E.SubElement(new, "uri").text = str(path)
        E.SubElement(new, "scale").text = f"{scale} {scale} {scale}"
(P / "models").mkdir(exist_ok=True)
E.indent(out)
E.ElementTree(out).write(P / "models/github_visuals.xml")
(P / "models/ADAPTATION.json").write_text(
    json.dumps(
        dict(
            model="ZX200",
            scale=scale,
            joints=angles,
            offset=[-0.12, 0.02, 0.1],
            visuals=len(out),
            source="vendor/zx200/SOURCE.json",
            physics="fixed mesh visual assembly; conservative body box collision; velocity control",
        ),
        indent=2,
    )
)
print("Imported", len(out), "textured ZX200 mesh parts from", source)
