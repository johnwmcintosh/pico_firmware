from machine import Pin
import time

class Encoder:
    def __init__(self, pin_a, pin_b, counts_per_lock=56, deg_per_lock=17.0):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)

        self.position = 0
        self._last_position = 0
        self._last_time = time.ticks_ms()
        self.velocity = 0

        # steering‑specific calibration 
        self.counts_per_lock = counts_per_lock
        self.deg_per_lock = deg_per_lock
        self.deg_per_count: float = deg_per_lock / counts_per_lock

        # drive motor calibration
        self.counts_per_rev = 199
        self.wheel_circ_m = 0.2136
        self.m_per_count = self.wheel_circ_m / self.counts_per_rev  # ≈ 0.001074

        self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._update)
        self.pin_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._update)

    def _update(self, pin):
        a = self.pin_a.value()
        b = self.pin_b.value()
        if a == b:
            self.position += 1
        else:
            self.position -= 1
        
        # For steering encoder, clamp position within limits of the steering apparatus
        self.clamp_position()

    def zero(self): 
        self.position = 0 
    
    def angle_deg(self): 
        return self.position * self.deg_per_count
    
    def angle_deg_clamped(self):
        angle = self.angle_deg()
        if angle > self.deg_per_lock:
            return self.deg_per_lock
        if angle < -self.deg_per_lock:
            return -self.deg_per_lock
        return angle
    
    def clamp_position(self):
        max_count = self.counts_per_lock
        if self.position > max_count:
            self.position = max_count
        elif self.position < -max_count:
            self.position = -max_count

    def get_position(self):
        return self.position

    def distance_m(self):
        return self.position * self.m_per_count

    def get_velocity(self):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._last_time)
        if dt >= 100:
            dp = self.position - self._last_position
            self.velocity = (dp * 1000) / dt
            self._last_position = self.position
            self._last_time = now
        return self.velocity
