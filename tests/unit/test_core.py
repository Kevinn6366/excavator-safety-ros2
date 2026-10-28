import unittest
import numpy as np
from excavator_safety.control.controller import Controller, level
from excavator_safety.perception.geometry import unproject, clearance
from excavator_safety.perception.detection import detect


class Tests(unittest.TestCase):
    def test_boundary(self):
        self.assertEqual(
            [level(x) for x in [1.5, 1.501, 3, 3.001, 6, 6.001]], [3, 2, 2, 1, 1, 0]
        )

    def test_fault_and_brake(self):
        c = Controller()
        c.v = 1
        for _ in range(20):
            s, v = c.step(20, 1, 0.05, False)
        self.assertEqual(s, 3)
        self.assertAlmostEqual(v, 0)

    def test_hysteresis(self):
        c = Controller()
        self.assertEqual(c.step(1.7, 1, 0.05)[0], 3)
        self.assertEqual(c.step(2, 1, 0.05)[0], 2)

    def test_projection(self):
        p = unproject(np.array([[2.0]]), [100, 100, 0, 0], "front", pitch=0)
        np.testing.assert_allclose(p[0, 0], [3.5, 0.55, 2.8])
        self.assertAlmostEqual(float(clearance(3.5, 0)), 2.2)

    def test_segmentation(self):
        rgb = np.zeros((80, 80, 3), np.uint8)
        rgb[25:65, 30:50] = [255, 0, 0]
        d = np.full((80, 80), 4.0, np.float32)
        objects, _ = detect(rgb, d, [100, 100, 40, 40], "front")
        self.assertEqual(len(objects), 1)
        self.assertEqual(objects[0]["label"], "person")

    def test_observed_boundary_regression(self):
        for measured in [1.5540902466222284, 1.5540895789466722]:
            c = Controller()
            self.assertEqual(c.step(measured - 0.15, 1.0, 0.05)[0], 3)

    def test_invalid_depth(self):
        o, _ = detect(
            np.full((20, 20, 3), [255, 0, 0], dtype=np.uint8),
            np.full((20, 20), np.nan),
            [100, 100, 10, 10],
            "rear",
        )
        self.assertEqual(o, [])


if __name__ == "__main__":
    unittest.main()


class DynamicBrakingTests(unittest.TestCase):
    def test_speed_extends_braking_threshold(self):
        slow = Controller()
        fast = Controller()
        slow.state = fast.state = 0
        self.assertNotEqual(slow.step(5.0, 3.0, 0.05, True, 0.0)[0], 3)
        self.assertEqual(fast.step(5.0, 3.0, 0.05, True, 3.0)[0], 3)
        self.assertAlmostEqual(fast.brake_threshold, 6.2)

    def test_feedback_overrides_low_command(self):
        c = Controller()
        c.state = 0
        c.v = 0
        self.assertEqual(c.step(4.0, 3.0, 0.05, True, 3.0)[0], 3)

    def test_free_space_reaches_requested_speed(self):
        c = Controller()
        for _ in range(180):
            c.step(99.0, 3.0, 0.05)
        self.assertAlmostEqual(c.v, 3.0)

    def test_brake_does_not_release_while_decelerating(self):
        c = Controller()
        c.state = 0
        c.v = 3.0
        d = 6.0
        for _ in range(65):
            state, v = c.step(d, 3.0, 0.05, True, c.v)
            d -= v * 0.05
            self.assertEqual(state, 3)
        self.assertAlmostEqual(v, 0.0)
        self.assertGreater(d, 0.0)
