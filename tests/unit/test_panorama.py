import unittest, numpy as np
from excavator_safety.visualization.panorama import Panorama


class PanoramaTests(unittest.TestCase):
    def test_cardinal_pixels_are_camera_pixels(self):
        colors = {
            "front": (0, 0, 255),
            "left": (0, 255, 0),
            "rear": (255, 0, 0),
            "right": (0, 255, 255),
        }
        frames = {s: np.full((240, 320, 3), c, np.uint8) for s, c in colors.items()}
        K = {s: [160 / np.tan(1.1), 160 / np.tan(1.1), 160, 120] for s in colors}
        im, coverage = Panorama().stitch(frames, K)
        self.assertEqual(im.shape, (160, 960, 3))
        self.assertGreater(coverage, 0.95)
        for x, s in [(0, "rear"), (240, "left"), (480, "front"), (720, "right")]:
            np.testing.assert_allclose(im[80, x], colors[s], atol=1)

    def test_pixels_change_with_sensor_image(self):
        frames = {
            s: np.zeros((240, 320, 3), np.uint8)
            for s in ["front", "left", "rear", "right"]
        }
        K = {s: [160 / np.tan(1.1), 160 / np.tan(1.1), 160, 120] for s in frames}
        p = Panorama()
        a, _ = p.stitch(frames, K)
        frames["front"][:] = 255
        b, _ = p.stitch(frames, K)
        self.assertEqual(int(a[80, 480].sum()), 0)
        self.assertGreater(int(b[80, 480].sum()), 750)


if __name__ == "__main__":
    unittest.main()
