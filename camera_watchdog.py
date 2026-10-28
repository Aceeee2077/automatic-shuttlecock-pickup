"""Camera stale-stream detection.

A USB camera can stop producing new frames without `cap.read()` ever
returning False: the driver keeps returning the last buffered frame. The
main loop would then keep seeing the same old target and drive toward it.

`FrameWatchdog` timestamps every new frame and answers whether the stream
has gone quiet for longer than a timeout. It uses a caller-supplied clock
so it can be tested without sleeping or opening a camera.
"""

import time


class FrameWatchdog:
    def __init__(self, timeout, clock=None):
        timeout = float(timeout)

        if timeout <= 0:
            raise ValueError("timeout must be positive")

        self.timeout = timeout
        self._clock = clock or time.time
        self.last_frame_at = None
        self.frames = 0
        self.stale_events = 0
        self._stale = False

    def mark_frame(self):
        """Record that a new frame arrived. Returns True when recovering."""
        self.last_frame_at = self._clock()
        self.frames += 1

        recovered = self._stale
        self._stale = False
        return recovered

    def frame_age(self):
        """Seconds since the last new frame, or None before the first one."""
        if self.last_frame_at is None:
            return None

        return max(0.0, self._clock() - self.last_frame_at)

    def is_stale(self):
        """True when no new frame arrived within the timeout."""
        age = self.frame_age()

        if age is None or age <= self.timeout:
            self._stale = False
            return False

        if not self._stale:
            self._stale = True
            self.stale_events += 1

        return True
