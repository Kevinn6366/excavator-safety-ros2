"""Render only the existing ZX200 mesh as a transparent orthographic UI asset.
This renderer never reads simulated environment entities or camera streams.
"""

from excavator_safety.paths import ROOT

from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import cv2
from scipy.spatial.transform import Rotation

P = ROOT
N = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
size = 720
extent = 3.0
scale = size / (2 * extent)
image = np.zeros((size, size, 4), np.uint8)
zbuffer = np.full((size, size), -np.inf)
texture = cv2.imread(str(P / "vendor/zx200/meshes/bake.png"))
th, tw = texture.shape[:2]


def raster(points, uv):
    screen = np.stack(
        (size / 2 - points[:, 1] * scale, size / 2 - points[:, 0] * scale), axis=1
    )
    lo = np.maximum(np.floor(screen.min(axis=0)).astype(int), 0)
    hi = np.minimum(np.ceil(screen.max(axis=0)).astype(int), size - 1)
    if np.any(hi < lo):
        return
    a, b, c = screen
    den = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
    if abs(den) < 0.03:
        return
    yy, xx = np.mgrid[lo[1] : hi[1] + 1, lo[0] : hi[0] + 1]
    xx = xx + 0.5
    yy = yy + 0.5
    w0 = ((b[1] - c[1]) * (xx - c[0]) + (c[0] - b[0]) * (yy - c[1])) / den
    w1 = ((c[1] - a[1]) * (xx - c[0]) + (a[0] - c[0]) * (yy - c[1])) / den
    w2 = 1 - w0 - w1
    z = w0 * points[0, 2] + w1 * points[1, 2] + w2 * points[2, 2]
    zs = zbuffer[lo[1] : hi[1] + 1, lo[0] : hi[0] + 1]
    valid = (w0 >= 0) & (w1 >= 0) & (w2 >= 0) & (z > zs)
    if not valid.any():
        return
    coords = w0[:, :, None] * uv[0] + w1[:, :, None] * uv[1] + w2[:, :, None] * uv[2]
    tx = np.clip((coords[:, :, 0] * tw).astype(int), 0, tw - 1)
    ty = np.clip(((1 - coords[:, :, 1]) * th).astype(int), 0, th - 1)
    normal = np.cross(points[1] - points[0], points[2] - points[0])
    normal /= max(np.linalg.norm(normal), 1e-8)
    light = 0.65 + 0.35 * abs(normal @ np.array([0.25, -0.35, 0.902]))
    colors = (texture[ty, tx] * light).clip(0, 255).astype(np.uint8)
    patch = image[lo[1] : hi[1] + 1, lo[0] : hi[0] + 1]
    patch[valid, :3] = colors[valid]
    patch[valid, 3] = 255
    zs[valid] = z[valid]


triangles = 0
for vis in E.parse(P / "models/github_visuals.xml").getroot():
    pose = np.fromstring(vis.findtext("pose"), sep=" ")
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", pose[3:]).as_matrix() * 0.4
    T[:3, 3] = pose[:3]
    path = P / "vendor/zx200/meshes" / Path(vis.findtext("geometry/mesh/uri")).name
    root = E.parse(path).getroot()
    geometries = {
        g.get("id"): g for g in root.findall("c:library_geometries/c:geometry", N)
    }

    def visit(node, parent):
        global triangles
        local = np.eye(4)
        for mat in node.findall("c:matrix", N):
            local = local @ np.fromstring(mat.text, sep=" ").reshape(4, 4)
        transform = parent @ local
        for inst in node.findall("c:instance_geometry", N):
            mesh = geometries[inst.get("url")[1:]].find("c:mesh", N)
            sources = {}
            for src in mesh.findall("c:source", N):
                arr = src.find("c:float_array", N)
                acc = src.find("c:technique_common/c:accessor", N)
                if arr is not None:
                    sources[src.get("id")] = np.fromstring(arr.text, sep=" ").reshape(
                        -1, int(acc.get("stride", "1"))
                    )
            vertices = {
                v.get("id"): v.find("c:input", N).get("source")[1:]
                for v in mesh.findall("c:vertices", N)
            }
            for tri in mesh.findall("c:triangles", N):
                inputs = tri.findall("c:input", N)
                stride = max(int(i.get("offset", "0")) for i in inputs) + 1
                indices = np.fromstring(
                    tri.findtext("c:p", namespaces=N), sep=" ", dtype=int
                ).reshape(-1, 3, stride)
                vi = next(i for i in inputs if i.get("semantic") == "VERTEX")
                ti = next((i for i in inputs if i.get("semantic") == "TEXCOORD"), None)
                pts = sources[vertices[vi.get("source")[1:]]]
                pts = pts @ transform[:3, :3].T + transform[:3, 3]
                for ids in indices:
                    poly = pts[ids[:, int(vi.get("offset", "0"))]]
                    uv = (
                        sources[ti.get("source")[1:]][
                            ids[:, int(ti.get("offset", "0"))], :2
                        ]
                        if ti is not None
                        else np.zeros((3, 2))
                    )
                    raster(poly, uv)
                    triangles += 1
        for child in node.findall("c:node", N):
            visit(child, transform)

    for node in root.findall("c:library_visual_scenes/c:visual_scene/c:node", N):
        visit(node, T)
asset = P / "frontend/assets/zx200-roof.png"
cv2.imwrite(str(asset), image)
print(
    f"Rendered {triangles} triangles to {asset}; {np.count_nonzero(image[:,:,3])} visible pixels"
)
