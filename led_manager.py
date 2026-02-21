# led_status.py

import time
from machine import Pin

class LEDStatus:
    def __init__(self, pin=25):
        self.led = Pin(pin, Pin.OUT)
        self.last_toggle = time.ticks_ms()
        self.mode = "heartbeat"

    def on(self):
        self.led.value(1)

    def off(self):
        self.led.value(0)

    def toggle(self):
        self.led.toggle()

    def set_heartbeat(self):
        self.mode = "heartbeat"

    def set_watchdog(self):
        self.mode = "watchdog"

    def set_error(self):
        self.mode = "error"

    def update(self):
        now = time.ticks_ms()

        if self.mode == "heartbeat":
            if time.ticks_diff(now, self.last_toggle) > 500:
                self.toggle()
                self.last_toggle = now

        elif self.mode == "watchdog":
            if time.ticks_diff(now, self.last_toggle) > 100:
                self.toggle()
                self.last_toggle = now

        elif self.mode == "error":
            elapsed = time.ticks_diff(now, self.last_toggle)

            if elapsed > 200:
                self.toggle()
                self.last_toggle = now

                if hasattr(self, "_error_count"):
                    self._error_count += 1
                else:
                    self._error_count = 1

                if self._error_count >= 6:
                    time.sleep(1.0)
                    self._error_count = 0


def startup_blink(led: LEDStatus, mode: str):
    if mode == "DEBUG":
        for _ in range(2):
            led.on()
            time.sleep(0.2)
            led.off()
            time.sleep(0.2)
    else:
        for _ in range(3):
            led.on()
            time.sleep(0.1)
            led.off()
            time.sleep(0.1)


def enter_error_mode(led: LEDStatus):
    led.set_error()
