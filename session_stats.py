"""In-memory session statistics.

The robot only ever showed a live pickup count. Tuning a run needs the rest
of the picture: how long it has been running, how many pickup attempts it
made, how many succeeded, how often it escaped being stuck, and how many
targets were abandoned because of obstacles.

This module keeps those counters in memory and has no dependency on OpenCV,
YOLO or pyserial, so it can be tested offline and reused by the web monitor
and the shutdown summary.
"""

import time
from collections import deque


class SessionStats:
    """Counters and a rolling frame-rate estimate for one run.

    `clock` is injectable so tests do not need to sleep.
    """

    def __init__(self, fps_window=30, clock=None):
        self._clock = clock or time.time
        self.started_at = self._clock()
        self._frame_times = deque(maxlen=max(2, int(fps_window)))

        self.frames = 0
        self.collect_attempts = 0
        self.collect_successes = 0
        self.collect_failures = 0
        self.escapes = 0
        self.obstacle_giveups = 0
        self.target_switches = 0

    # ---- per-frame ----

    def tick(self):
        """Call once per processed frame."""
        self.frames += 1
        self._frame_times.append(self._clock())

    # ---- events ----

    def record_collect_start(self):
        self.collect_attempts += 1

    def record_collect_success(self):
        self.collect_successes += 1

    def record_collect_failure(self):
        self.collect_failures += 1

    def record_escape(self):
        self.escapes += 1

    def record_obstacle_giveup(self):
        self.obstacle_giveups += 1

    def record_target_switch(self):
        self.target_switches += 1

    # ---- derived values ----

    def uptime(self):
        return max(0.0, self._clock() - self.started_at)

    def fps(self):
        """Frames per second over the rolling window (0.0 until two frames)."""
        if len(self._frame_times) < 2:
            return 0.0

        span = self._frame_times[-1] - self._frame_times[0]

        if span <= 0:
            return 0.0

        return (len(self._frame_times) - 1) / span

    def success_rate(self):
        """Pickup success rate in percent; 0.0 before any attempt finishes."""
        finished = self.collect_successes + self.collect_failures

        if finished <= 0:
            return 0.0

        return 100.0 * self.collect_successes / finished

    def snapshot(self):
        """Plain dict for the web status endpoint and logging."""
        return {
            "uptime_s": round(self.uptime(), 1),
            "fps": round(self.fps(), 1),
            "frames": self.frames,
            "collect_attempts": self.collect_attempts,
            "collect_success": self.collect_successes,
            "collect_failed": self.collect_failures,
            "success_rate": round(self.success_rate(), 1),
            "escapes": self.escapes,
            "obstacle_giveups": self.obstacle_giveups,
            "target_switches": self.target_switches,
        }

    def summary_lines(self):
        """Human-readable shutdown summary."""
        data = self.snapshot()

        return [
            "Session summary",
            "  uptime: %.1f s (%.1f fps, %d frames)"
            % (data["uptime_s"], data["fps"], data["frames"]),
            "  pickups: %d/%d succeeded (%.1f%%), %d failed"
            % (
                data["collect_success"],
                data["collect_attempts"],
                data["success_rate"],
                data["collect_failed"],
            ),
            "  escapes: %d, obstacle give-ups: %d, target switches: %d"
            % (
                data["escapes"],
                data["obstacle_giveups"],
                data["target_switches"],
            ),
        ]

    def print_summary(self, stream=None):
        import sys

        stream = stream or sys.stdout

        for line in self.summary_lines():
            print(line, file=stream)
