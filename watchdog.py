# watchdog.py

import time

class Watchdog:
    def __init__(self, timeout_ms=2000):
        self.timeout_ms = timeout_ms
        self.last_reset = time.ticks_ms()
        self._armed = False

    def start(self):
        self.last_reset = time.ticks_ms()
        self._armed = True

    def reset(self):
        if self._armed:
            self.last_reset = time.ticks_ms()

    def check(self):
        if not self._armed:
            return

        if time.ticks_diff(time.ticks_ms(), self.last_reset) > self.timeout_ms:
            raise Exception("Watchdog timeout")
