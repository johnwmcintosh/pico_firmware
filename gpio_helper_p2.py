import time
import gpiod
from machine import Pin, PWM

class GPIOPin:
    def __init__(self, chip_name, line_offset, direction="out", default=0):
        self.chip = gpiod.Chip(chip_name)
        self.line = self.chip.get_line(line_offset)

        if direction == "out":
            request_type = gpiod.LINE_REQ_DIR_OUT
        else:
            request_type = gpiod.LINE_REQ_DIR_IN

        self.line.request(consumer="lidar_ros2", type=request_type, default_vals=[default])

    def set(self, value: int):
        self.line.set_value(1 if value else 0)

    def get(self) -> int:
        return self.line.get_value()

# DRV8871 Motor Driver Class for Raspberry Pi Pico with PWM control
class DRV8871:
    """
    Clean, safe DRV8871 driver for the Pico.
    - PWM is applied to only one pin at a time (required by DRV8871)
    - Direction changes include dead-time to prevent shoot-through
    - set_speed() provides a unified signed-speed interface
    """

    def __init__(self, in1_pin, in2_pin, pwm_freq=20000):
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.pwm_freq = pwm_freq

        # Initialize pins
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)

        # Start with PWM on IN1 by default
        self.pwm = PWM(self.in1)
        self.pwm.freq(self.pwm_freq)

        # Track which pin currently has PWM
        self.active_pwm_pin = self.in1

        self.stop()

    # ------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------

    def _apply_pwm_to_pin(self, pin, duty):
        """
        Safely move PWM to a new pin.
        DRV8871 requires PWM on only one input at a time.
        """
        if self.active_pwm_pin != pin:
            # Disable PWM on the old pin
            self.pwm.deinit()

            # Rebind PWM to the new pin
            self.pwm = PWM(pin)
            self.pwm.freq(self.pwm_freq)
            self.active_pwm_pin = pin

        self.pwm.duty_u16(duty)

    def _dead_time(self):
        """Small delay to prevent shoot-through when switching direction."""
        time.sleep_us(10)

    # ------------------------------------------------------------
    # Public motor control API
    # ------------------------------------------------------------

    def forward(self, duty=65535):
        """
        Forward = PWM on IN1, IN2 held low.
        """
        self.in2.low()
        self._dead_time()
        self._apply_pwm_to_pin(self.in1, duty)

    def reverse(self, duty=65535):
        """
        Reverse = PWM on IN2, IN1 held low.
        """
        self.in1.low()
        self._dead_time()
        self._apply_pwm_to_pin(self.in2, duty)

    def stop(self):
        """Coast stop (both inputs low)."""
        self.pwm.duty_u16(0)
        self.in1.low()
        self.in2.low()

    # ------------------------------------------------------------
    # Unified signed-speed interface
    # ------------------------------------------------------------

    def set_speed(self, value):
        """
        value: signed integer from -100 to +100
        """
        # Clamp
        value = max(min(value, 100), -100)

        if value == 0:
            self.stop()
            return

        duty = int(abs(value) * 65535 / 100)

        if value > 0:
            self.forward(duty)
        else:
            self.reverse(duty)
