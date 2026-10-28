import unittest

from camera_watchdog import FrameWatchdog


class FakeClock:
    def __init__(self, start=100.0):
        self.now = start

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FrameWatchdogTests(unittest.TestCase):
    def make(self, timeout=2.0):
        clock = FakeClock()
        return FrameWatchdog(timeout, clock=clock), clock

    def test_healthy_before_first_frame(self):
        watchdog, _ = self.make()

        self.assertFalse(watchdog.is_stale())
        self.assertIsNone(watchdog.frame_age())
        self.assertEqual(watchdog.frames, 0)

    def test_frame_age_tracks_clock(self):
        watchdog, clock = self.make()

        watchdog.mark_frame()
        clock.advance(0.75)

        self.assertAlmostEqual(watchdog.frame_age(), 0.75)
        self.assertFalse(watchdog.is_stale())

    def test_stale_after_timeout(self):
        watchdog, clock = self.make(timeout=2.0)

        watchdog.mark_frame()
        clock.advance(2.0)
        self.assertFalse(watchdog.is_stale())

        clock.advance(0.1)
        self.assertTrue(watchdog.is_stale())

    def test_new_frame_recovers(self):
        watchdog, clock = self.make(timeout=1.0)

        watchdog.mark_frame()
        clock.advance(5.0)
        self.assertTrue(watchdog.is_stale())

        self.assertTrue(watchdog.mark_frame())
        self.assertFalse(watchdog.is_stale())
        self.assertEqual(watchdog.frames, 2)
        self.assertEqual(watchdog.stale_events, 1)

    def test_stale_events_count_episodes(self):
        watchdog, clock = self.make(timeout=1.0)
        watchdog.mark_frame()

        clock.advance(2.0)
        self.assertTrue(watchdog.is_stale())
        self.assertTrue(watchdog.is_stale())

        watchdog.mark_frame()
        clock.advance(2.0)
        self.assertTrue(watchdog.is_stale())

        self.assertEqual(watchdog.stale_events, 2)

    def test_mark_frame_while_healthy_is_not_recovery(self):
        watchdog, _ = self.make()

        watchdog.mark_frame()
        self.assertFalse(watchdog.mark_frame())

    def test_timeout_must_be_positive(self):
        with self.assertRaises(ValueError):
            FrameWatchdog(0)


if __name__ == "__main__":
    unittest.main()
