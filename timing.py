import time

class Metronome:
    """Simple timing class."""

    def __init__(self, freq):
        """Initialize metronome with given frequency.

        Args:
            freq: Frequency in Hz to tick at.
        """
        self._tick_t = 1/freq
        self._t0 = None
        self._next_t = None
        self.reset()

    def reset(self):
        """Reset metronome timing to zero."""
        self._t0 = time.time()
        self._next_t = self._tick_t
        self._tick_i = 0

    @property
    def t(self):
        """Get current time since start in seconds."""
        return time.time() - self._t0

    @property
    def i(self):
        """Get current tick count."""
        return self._tick_i

    def wait(self, catch_up=False):
        """Wait for next beat.

        Args:
            catch_up: If True, skips beats to catch up if running behind.
                   If False, maintains fixed intervals even if running behind.

        Returns:
            Current time since start in seconds.
        """
        while self.t < self._next_t:
            time.sleep(self._tick_t / 10)
        if catch_up:
            self._next_t += self._tick_t
        else:
            self._next_t = self.t + self._tick_t
        self._tick_i += 1
        return self.t 