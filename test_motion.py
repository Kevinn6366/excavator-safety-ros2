import unittest
from core import Controller, slew_signed


class MotionTests(unittest.TestCase):
    def test_linear_acceleration_both_directions(self):
        for sign in [1, -1]:
            c = Controller()
            last = 0.0
            for _ in range(20):
                _, v = c.step(99, sign * 3, 0.05)
                self.assertAlmostEqual(v - last, sign * 0.4 * 0.05)
                last = v
            self.assertAlmostEqual(v, sign * 0.4)

    def test_reverse_stops_before_direction_change(self):
        c = Controller()
        c.v = 1.0
        c.state = 0
        values = []
        for _ in range(40):
            values.append(c.step(99, -2, 0.05)[1])
        first = next(i for i, v in enumerate(values) if v < 0)
        self.assertAlmostEqual(values[first - 1], 0.0)
        self.assertGreaterEqual(values[first], -0.020001)

    def test_reverse_aeb_stops_linear_and_angular_motion(self):
        c = Controller()
        c.state = 0
        c.v = -2.0
        c.w = -0.4
        for _ in range(45):
            state, v = c.step(1.0, -3, 0.05, True, c.v, -0.55, c.w)
        self.assertEqual(state, 3)
        self.assertAlmostEqual(v, 0)
        self.assertAlmostEqual(c.w, 0)

    def test_turning_extends_corner_stopping_distance(self):
        straight = Controller()
        turn = Controller()
        straight.state = turn.state = 0
        self.assertNotEqual(straight.step(6.5, 3, 0.05, True, 3, 0, 0)[0], 3)
        self.assertEqual(turn.step(6.5, 3, 0.05, True, 3, 0.55, 0.55)[0], 3)
        self.assertGreater(turn.brake_threshold, straight.brake_threshold)

    def test_pivot_and_key_release(self):
        c = Controller()
        for _ in range(30):
            c.step(99, 0, 0.05, True, 0, 0.55, 0)
        self.assertAlmostEqual(c.w, 0.55)
        self.assertEqual(c.v, 0)
        for _ in range(12):
            c.step(99, 0, 0.05, True, 0, 0, 0)
        self.assertAlmostEqual(c.w, 0)

    def test_fail_safe_stops_reverse_turn(self):
        c = Controller()
        c.state = 0
        c.v = -1
        c.w = 0.5
        for _ in range(30):
            c.step(99, -3, 0.05, False, c.v, 0.55, c.w)
        self.assertEqual(c.state, 3)
        self.assertAlmostEqual(c.v, 0)
        self.assertAlmostEqual(c.w, 0)


class HighSpeedTests(unittest.TestCase):
    def test_full_speed_and_finite_braking(self):
        for direction in [-1, 1]:
            c = Controller()
            for _ in range(800):
                c.step(float("inf"), direction * 50 / 3.6, 0.05)
            self.assertAlmostEqual(c.v, direction * 50 / 3.6)
            c.step(1, direction * 50 / 3.6, 0.05)
            self.assertGreater(abs(c.v), 13)
            self.assertEqual(c.state, 3)
