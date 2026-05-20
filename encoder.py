from machine import Pin
import time

# ---------------------------------------------------------
# DRIVING ENCODER (for wheel odometry)
# ---------------------------------------------------------
class DrivingEncoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)

        self.position = 0
        self._last_position = 0
        self._last_time = time.ticks_ms()
        self.velocity = 0

        # wheel odometry calibration
        self.counts_per_rev = 204
        self.wheel_circ_m = 0.2136  # 67mm diameter = 0.2136m circumference
        self.m_per_count = self.wheel_circ_m / self.counts_per_rev

        # quadrature on A only (fine for drive wheels)
        self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                       handler=self._update)

    def _update(self, pin):
        a = self.pin_a.value()
        b = self.pin_b.value()
        prev = self.position
        
        if a == b:
            self.position -= 1
        else:
            self.position += 1

        # Reject impossible jumps (noise)
        if abs(self.position - prev) > 5:
            self.position = prev

    def zero(self):
        self.position = 0

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


# ---------------------------------------------------------
# STEERING ENCODER (normalized angle, ±max_count)
# ---------------------------------------------------------
class SteeringEncoder:
    def __init__(self, pin_a, pin_b, max_count=11):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)

        self.position = 0
        self.max_count = max_count  # updated dynamically by smart auto-zero

        # quadrature on BOTH channels for steering precision
        self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                       handler=self._update)
        self.pin_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                       handler=self._update)

    def _update(self, pin):
        a = self.pin_a.value()
        b = self.pin_b.value()

        if a == b:
            self.position -= 1
        else:
            self.position += 1

        # clamp to physical limits
        if self.position > self.max_count:
            self.position = self.max_count
        elif self.position < -self.max_count:
            self.position = -self.max_count

    def zero(self):
        self.position = 0

    def get_position(self):
        return self.position

    def get_angle(self):
        """
        Normalized steering angle:
        -1.0 = full right
         0.0 = center
        +1.0 = full left
        """
        if self.max_count == 0:
            return 0.0
        return self.position / self.max_count
