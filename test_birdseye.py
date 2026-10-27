import unittest
import numpy as np
from birdseye import Birdseye
from core import transform


def scene():
    h, w = 240, 320
    K = [81.4, 81.4, 160, 120]
    v, u = np.indices((h, w))
    rays = np.stack(((u - 160) / K[0], (v - 120) / K[1], np.ones((h, w))), axis=-1)
    R, t = transform("front")
    ray = rays @ R.T
    with np.errstate(divide="ignore", invalid="ignore"):
        ground = -t[2] / ray[:, :, 2]
    ground = np.where(ground > 0.1, ground, np.inf)
    low = np.array([4.0, -0.4, 0.0])
    high = np.array([4.8, 0.4, 1.7])
    with np.errstate(divide="ignore", invalid="ignore"):
        a = (low - t) / ray
        b = (high - t) / ray
    enter = np.minimum(a, b).max(axis=2)
    leave = np.maximum(a, b).min(axis=2)
    hit = (leave >= enter) & (enter > 0.1) & (enter < ground)
    depth = np.where(hit, enter, ground).astype(np.float32)
    rgb = np.full((h, w, 3), 125, np.uint8)
    rgb[hit] = [25, 35, 230]
    return rgb, depth, K


class BirdseyeTests(unittest.TestCase):
    def test_elevated_object_is_not_stretched(self):
        rgb, depth, K = scene()
        b = Birdseye(600, 7)
        out, coverage = b.stitch({"front": rgb}, {"front": K}, {"front": depth})
        red = (out[:, :, 2] > 180) & (out[:, :, 0] < 70)
        v, u = np.where(red)
        self.assertGreater(len(v), 20)
        x = (300 - v) / b.scale
        y = (300 - u) / b.scale
        self.assertGreaterEqual(x.min(), 3.9)
        self.assertLessEqual(x.max(), 4.9)
        self.assertLessEqual(abs(y).max(), 0.5)
        self.assertGreater(coverage, 0.15)

    def test_no_depth_means_no_invented_ground(self):
        b = Birdseye(60)
        out, coverage = b.stitch(
            {"front": np.full((240, 320, 3), 255, np.uint8)},
            {"front": [80, 80, 160, 120]},
        )
        self.assertEqual(coverage, 0)
        np.testing.assert_array_equal(out, b.background)

    def test_source_colors_reach_ground(self):
        rgb, depth, K = scene()
        b = Birdseye(100)
        a, _ = b.stitch({"front": rgb}, {"front": K}, {"front": depth})
        rgb[:] = [70, 150, 90]
        changed, _ = b.stitch({"front": rgb}, {"front": K}, {"front": depth})
        self.assertGreater(np.abs(changed.astype(float) - a).sum(), 10000)
