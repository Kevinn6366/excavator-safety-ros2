import unittest
from core import Controller, assess_path


def target(x, y, side="front", label="person"):
    return dict(position=[x, y, 1.0], side=side, label=label)


class PathPolicyTests(unittest.TestCase):
    def test_front_obstacle_does_not_lock_reverse(self):
        ts = [target(2.0, 0)]
        self.assertLess(assess_path(ts, 0, 0, 1, 0)["distance"], 90)
        self.assertEqual(assess_path(ts, 0, 0, -1, 0)["distance"], 99)

    def test_rear_obstacle_does_not_lock_forward(self):
        ts = [target(-2.0, 0, "rear")]
        self.assertLess(assess_path(ts, 0, 0, -1, 0)["distance"], 90)
        self.assertEqual(assess_path(ts, 0, 0, 1, 0)["distance"], 99)

    def test_momentum_still_checked_while_reversing_request(self):
        self.assertLess(assess_path([target(2.2, 0)], 1, 0, -1, 0)["distance"], 90)

    def test_parallel_side_target_is_not_in_path(self):
        self.assertEqual(
            assess_path([target(0, 2.5, "left")], 1, 0, 1, 0)["distance"], 99
        )

    def test_curved_swept_body(self):
        ts = [target(2.2, 1.6, "left")]
        self.assertLess(assess_path(ts, 0.6, 0.4, 0.6, 0.4)["distance"], 90)
        self.assertEqual(assess_path(ts, -0.6, 0, -0.6, 0)["distance"], 99)

    def test_escape_from_existing_contact(self):
        self.assertEqual(assess_path([target(1.3, 0)], 0, 0, -0.5, 0)["distance"], 99)

    def test_coast_deceleration_is_linear(self):
        c = Controller()
        c.state = 0
        c.v = 1
        for _ in range(20):
            c.step(99, 0, 0.05)
        self.assertAlmostEqual(c.v, 0.75)

    def test_brake_is_stronger_than_coasting(self):
        c = Controller()
        c.state = 0
        c.v = -1
        for _ in range(20):
            c.step(99, 0, 0.05, braking=True)
        self.assertAlmostEqual(c.v, 0.0)

    def test_direction_change_releases_aeb(self):
        c = Controller(limits=(0.25, 0.8, 2))
        ts = [target(2.0, 0)]
        for _ in range(20):
            c.step(assess_path(ts, c.v, 0, 1, 0)["distance"] - 0.15, 1, 0.05)
        self.assertEqual(c.state, 3)
        distance = assess_path(ts, 0, 0, -1, 0)["distance"]
        c.step(distance, -1, 0.05)
        self.assertEqual(c.state, 0)
        self.assertLess(c.v, 0)

    def test_manual_brake_overrides_throttle_and_steering(self):
        c = Controller()
        c.state = 0
        c.v = 1.0
        c.w = 0.4
        for _ in range(20):
            c.step(99, 3, 0.05, requested_yaw=0.55, braking=True)
        self.assertAlmostEqual(c.v, 0.0)
        self.assertAlmostEqual(c.w, 0.0)
