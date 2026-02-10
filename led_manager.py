import time
from machine import Pin

class LEDStatus:
    def __init__(self, pin=25):
        self.led = Pin(pin, Pin.OUT)
        self.last_toggle = time.ticks_ms()
        self.mode = "heartbeat"

    def set_heartbeat(self):
        self.mode = "heartbeat"

    def set_watchdog(self):
        self.mode = "watchdog"

    def update(self):
        now = time.ticks_ms()

        if self.mode == "heartbeat":
            if time.ticks_diff(now, self.last_toggle) > 500:
                self.led.toggle()
                self.last_toggle = now

        elif self.mode == "watchdog":
            if time.ticks_diff(now, self.last_toggle) > 100:
                self.led.toggle()
                self.last_toggle = now
