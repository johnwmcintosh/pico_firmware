from machine import Timer
from led_manager import LEDStatus

class Watchdog:
    def __init__(self, timeout_ms=2000, callback=None, led_status: LEDStatus | None = None):
        self.timer = Timer()
        self.timeout_ms = timeout_ms
        self.callback = callback or self._default_callback
        self._armed = False
        self.led_status: LEDStatus = led_status or LEDStatus()

    def _default_callback(self, t):
        self.led_status.set_watchdog()
        print("Watchdog timeout – no callback provided")

    def start(self):
        self.timer.init(period=self.timeout_ms, mode=Timer.PERIODIC, callback=self.callback)
        self._armed = True

def reset(self):
    if self._armed:
        self.led_status.set_heartbeat()
        self.timer.init(period=self.timeout_ms, mode=Timer.PERIODIC, callback=self.callback)

    def stop(self):
        self.timer.deinit()
        self._armed = False
