from machine import Pin
import time

class Encoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)

        self.position = 0
        self._last_position = 0
        self._last_time = time.ticks_ms()
        self.velocity = 0

        self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._update)
        self.pin_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._update)

    def _update(self, pin):
        a = self.pin_a.value()
        b = self.pin_b.value()
        if a == b:
            self.position += 1
        else:
            self.position -= 1

    def get_position(self):
        return self.position

    def get_velocity(self):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._last_time)
        if dt >= 100:
            dp = self.position - self._last_position
            self.velocity = (dp * 1000) / dt
            self._last_position = self.position
            self._last_time = now
        return self.velocity
