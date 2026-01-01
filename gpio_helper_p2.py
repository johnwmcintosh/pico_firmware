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
    def __init__(self, in1_pin, in2_pin, pwm_freq=20000):
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm = PWM(self.in1)
        self.pwm.freq(pwm_freq)
        self.stop()

    def forward(self, duty=65535):
        self.in2.low()
        self.pwm.duty_u16(duty)

    def reverse(self, duty=65535):
        self.in1.low()
        self.in2.high()
        time.sleep_us(10)  # Ensure IN1 is low before enabling IN2
        self.in1.init(Pin.OUT)
        self.pwm = PWM(self.in2)
        self.pwm.freq(self.pwm.freq())
        self.pwm.duty_u16(duty)

    def stop(self):
        self.pwm.duty_u16(0)
        self.in1.low()
        self.in2.low()
