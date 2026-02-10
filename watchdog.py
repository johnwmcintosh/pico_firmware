import time
from led_manager import LEDStatus

class Watchdog:
    def __init__(self, timeout_ms=2000, led_status=None):
        self.timeout_ms = timeout_ms
        self.led_status = led_status or LEDStatus()
        self.last_reset = time.ticks_ms()
        self._armed = False

    def start(self):
        self.last_reset = time.ticks_ms()
        self._armed = True

    def reset(self):
        if self._armed:
            self.last_reset = time.ticks_ms()
            self.led_status.set_heartbeat()

    def check(self):
        if self._armed:
            if time.ticks_diff(time.ticks_ms(), self.last_reset) > self.timeout_ms:
                self.led_status.set_watchdog()
                print("Watchdog timeout!")
                # You can add motor stop logic here if needed
