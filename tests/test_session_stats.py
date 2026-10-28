import io
import unittest

from session_stats import SessionStats


class FakeClock:
    def __init__(self, start=1000.0):
        self.now = start

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class SessionStatsTests(unittest.TestCase):
    def make(self, fps_window=5):
        clock = FakeClock()
        return SessionStats(fps_window=fps_window, clock=clock), clock

    def test_starts_empty(self):
        stats, _ = self.make()
        data = stats.snapshot()

        self.assertEqual(data["frames"], 0)
        self.assertEqual(data["collect_attempts"], 0)
        self.assertEqual(data["collect_success"], 0)
        self.assertEqual(data["collect_failed"], 0)
        self.assertEqual(data["success_rate"], 0.0)
        self.assertEqual(data["uptime_s"], 0.0)
        self.assertEqual(data["fps"], 0.0)

    def test_uptime_follows_clock(self):
        stats, clock = self.make()
        clock.advance(12.5)

        self.assertAlmostEqual(stats.uptime(), 12.5)
        self.assertEqual(stats.snapshot()["uptime_s"], 12.5)

    def test_fps_needs_two_frames(self):
        stats, clock = self.make()

        stats.tick()
        self.assertEqual(stats.fps(), 0.0)

        clock.advance(0.1)
        stats.tick()
        self.assertAlmostEqual(stats.fps(), 10.0)

    def test_fps_uses_rolling_window(self):
        stats, clock = self.make(fps_window=3)

        for _ in range(10):
            stats.tick()
            clock.advance(0.1)

        # Only the last 3 frame times count: 2 intervals of 0.1 s = 10 fps.
        self.assertAlmostEqual(stats.fps(), 10.0)

    def test_fps_ignores_zero_span(self):
        stats, _ = self.make()
        stats.tick()
        stats.tick()

        self.assertEqual(stats.fps(), 0.0)

    def test_event_counters(self):
        stats, _ = self.make()

        stats.record_collect_start()
        stats.record_collect_start()
        stats.record_collect_success()
        stats.record_collect_failure()
        stats.record_escape()
        stats.record_obstacle_giveup()
        stats.record_target_switch()

        data = stats.snapshot()
        self.assertEqual(data["collect_attempts"], 2)
        self.assertEqual(data["collect_success"], 1)
        self.assertEqual(data["collect_failed"], 1)
        self.assertEqual(data["escapes"], 1)
        self.assertEqual(data["obstacle_giveups"], 1)
        self.assertEqual(data["target_switches"], 1)

    def test_success_rate_uses_finished_attempts(self):
        stats, _ = self.make()

        self.assertEqual(stats.success_rate(), 0.0)

        stats.record_collect_start()
        stats.record_collect_success()
        stats.record_collect_success()
        stats.record_collect_failure()

        # Two successes out of three finished attempts, not three starts.
        self.assertAlmostEqual(stats.success_rate(), 200.0 / 3.0)
        self.assertEqual(stats.snapshot()["success_rate"], 66.7)

    def test_summary_lines_report_the_session(self):
        stats, clock = self.make()
        clock.advance(65)
        stats.record_collect_start()
        stats.record_collect_success()
        stats.record_escape()

        out = io.StringIO()
        stats.print_summary(stream=out)
        text = out.getvalue()

        self.assertIn("Session summary", text)
        self.assertIn("uptime: 65.0 s", text)
        self.assertIn("pickups: 1/1 succeeded (100.0%), 0 failed", text)
        self.assertIn("escapes: 1", text)

    def test_snapshot_rounds_display_values(self):
        stats, clock = self.make()
        stats.tick()
        clock.advance(0.1)
        stats.tick()

        data = stats.snapshot()
        self.assertEqual(data["uptime_s"], 0.1)
        self.assertEqual(data["fps"], 10.0)


if __name__ == "__main__":
    unittest.main()
