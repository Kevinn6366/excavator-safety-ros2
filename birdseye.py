"""RGB-D orthographic surround view. No world poses or scene-truth inputs.
Ground pixels are inverse-projected only when measured depth confirms ground.
Above-ground surfaces are splatted at their measured XYZ, preventing tall-object
stretch. Unobserved regions remain a neutral hatch, never invented imagery.
"""

import cv2
import numpy as np
from core import transform, unproject


class Birdseye:
    def __init__(self, size=600, extent=7.0):
        self.size = size
        self.extent = extent
        self.scale = size / (2 * extent)
        v, u = np.indices((size, size))
        self.xyz = np.stack(
            (
                (size / 2 - v) / self.scale,
                (size / 2 - u) / self.scale,
                np.zeros_like(u),
            ),
            axis=-1,
        )
        shade = np.where((u % 12 < 2) & (v % 12 < 2), 132, 145).astype(np.uint8)
        self.background = np.stack((shade, shade, shade), axis=-1)

    def stitch(self, frames, intrinsics, depths=None):
        depths = depths or {}
        size = self.size
        total = np.zeros((size, size, 3), np.float32)
        weight = np.zeros((size, size), np.float32)
        clouds = []
        colors = []
        for side, bgr in frames.items():
            if side not in depths or side not in intrinsics:
                continue
            depth = depths[side]
            R, t = transform(side)
            opt = (self.xyz - t) @ R
            z = opt[:, :, 2]
            fx, fy, cx, cy = intrinsics[side]
            safe = np.where(z > 0.1, z, 1)
            mx = (fx * opt[:, :, 0] / safe + cx).astype(np.float32)
            my = (fy * opt[:, :, 1] / safe + cy).astype(np.float32)
            pts = unproject(depth, intrinsics[side], side)
            h, w = bgr.shape[:2]
            valid_depth = np.isfinite(depth) & (depth > 0.1) & (depth < 15)
            ground = (valid_depth & (np.abs(pts[:, :, 2]) < 0.15)).astype(np.uint8)
            # Erode at object edges so interpolation cannot smear object colors onto ground.
            ground = cv2.erode(ground, np.ones((3, 3), np.uint8))
            confirmed = (
                cv2.remap(
                    ground, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT
                )
                > 0
            )
            valid = (
                (z > 0.1)
                & (z < 15)
                & (mx >= 0)
                & (mx < w - 1)
                & (my >= 0)
                & (my < h - 1)
                & confirmed
            )
            feather = (
                np.clip(np.minimum.reduce([mx, w - 1 - mx, my, h - 1 - my]) / 12, 0, 1)
                * valid
            )
            pixels = cv2.remap(
                bgr, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
            )
            total += pixels * feather[:, :, None]
            weight += feather
            elevated = valid_depth & (pts[:, :, 2] >= 0.15) & (pts[:, :, 2] < 4.0)
            q = pts[elevated]
            c = bgr[elevated]
            inside = (np.abs(q[:, 0]) < self.extent) & (np.abs(q[:, 1]) < self.extent)
            clouds.append(q[inside])
            colors.append(c[inside])
        out = self.background.copy()
        known = weight > 0
        out[known] = (total[known] / weight[known, None]).clip(0, 255).astype(np.uint8)
        if clouds and sum(map(len, clouds)):
            xyz = np.concatenate(clouds)
            rgb = np.concatenate(colors)
            x = np.clip(
                np.rint(size / 2 - xyz[:, 1] * self.scale).astype(int), 0, size - 1
            )
            y = np.clip(
                np.rint(size / 2 - xyz[:, 0] * self.scale).astype(int), 0, size - 1
            )
            # Highest observed surface wins per orthographic pixel. Tiny 5x5 splats cover
            # raster sampling gaps (12 cm), without stretching a person along a ground ray.
            flat = y * size + x
            order = np.lexsort((xyz[:, 2], flat))
            sorted_ids = flat[order]
            select = order[np.r_[sorted_ids[1:] != sorted_ids[:-1], True]]
            zbuf = np.full((size, size), -np.inf)
            for dy in [-2, -1, 0, 1, 2]:
                for dx in [-2, -1, 0, 1, 2]:
                    xx = np.clip(x[select] + dx, 0, size - 1)
                    yy = np.clip(y[select] + dy, 0, size - 1)
                    zz = xyz[select, 2]
                    ok = zz > zbuf[yy, xx]
                    out[yy[ok], xx[ok]] = rgb[select[ok]]
                    zbuf[yy[ok], xx[ok]] = zz[ok]
                    known[yy[ok], xx[ok]] = True
        return out, float(known.mean())
