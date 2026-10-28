"""360-degree spherical RGB panorama from four calibrated camera images.
Only image pixels, CameraInfo intrinsics and fixed mounting rotations are used.
Rotation-only stitching: near-field parallax is expected with separated cameras.
"""

import cv2, numpy as np
from excavator_safety.config import SIDES
from excavator_safety.perception.geometry import transform


class Panorama:
    def __init__(self, width=960, height=160):
        self.width = width
        self.height = height
        self.maps = {}

    def mapping(self, side, K, shape):
        key = (side, tuple(K), tuple(shape))
        if key in self.maps:
            return self.maps[key]
        az = np.pi - (np.arange(self.width) + 0.5) / self.width * 2 * np.pi
        elevation = np.deg2rad(10 - (np.arange(self.height) + 0.5) / self.height * 60)
        aa, ee = np.meshgrid(az, elevation)
        rays = np.stack(
            [np.cos(ee) * np.cos(aa), np.cos(ee) * np.sin(aa), np.sin(ee)], axis=-1
        )
        R, _ = transform(side)
        optical = rays @ R
        z = optical[:, :, 2]
        safe_z = np.where(z > 0.01, z, 1)
        u = (K[0] * optical[:, :, 0] / safe_z + K[2]).astype(np.float32)
        v = (K[1] * optical[:, :, 1] / safe_z + K[3]).astype(np.float32)
        h, w = shape[:2]
        valid = (z > 0.01) & (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
        weight = (np.maximum(0, 1 - ((u - K[2]) / (w / 2)) ** 2) ** 4) * valid
        self.maps[key] = (u, v, weight)
        return u, v, weight

    def stitch(self, frames, intrinsics):
        total = np.zeros((self.height, self.width, 3), np.float32)
        weights = np.zeros((self.height, self.width), np.float32)
        for side in SIDES:
            u, v, w = self.mapping(side, intrinsics[side], frames[side].shape)
            sample = cv2.remap(
                frames[side], u, v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
            )
            total += sample * w[:, :, None]
            weights += w
        image = np.clip(total / np.maximum(weights[:, :, None], 1e-6), 0, 255).astype(
            np.uint8
        )
        return image, float((weights > 1e-5).mean())
