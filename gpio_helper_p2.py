from machine import Pin, PWM

VERBOSE = False

def dbg(*args, **kwargs):
    if VERBOSE:
        print(*args, **kwargs)

class DRV8871:
    def __init__(self, pin_dir, pin_en):
        # pin_en -> IN1 (PWM)
        # pin_dir -> IN2 (direction)
        self.in2 = Pin(pin_dir, Pin.OUT)     # Direction
        self.pwm = PWM(Pin(pin_en))          # Speed via PWM on IN1
        self.pwm.freq(20000)                 # 20 kHz is a good quiet value

        self.MIN_DUTY = 59
        self.MAX_DUTY = 100
        self.MIN_FORWARD_PWM = 0.18 # pct of max duty for forward motion
        self.MIN_REVERSE_PWM = 0.18 # pct of max duty for reverse motion (reverse needs more power to overcome friction)

        self.stop()

    def set_power(self, speed):
        # Clamp input to [-1.0, 1.0]
        speed = max(min(speed, 1.0), -1.0)

        if speed == 0:
            self.pwm.duty_u16(0)
            self.in2.value(0)
            return

        # Determine direction and duty
        if speed > 0:
            duty_percent = max(abs(speed), self.MIN_FORWARD_PWM)
            self.in2.value(0)   # forward
            direction = "FWD"
        else:
            duty_percent = max(abs(speed), self.MIN_REVERSE_PWM)
            self.in2.value(1)   # reverse
            direction = "REV"

        # Convert percent → 16‑bit PWM
        pwm = int(duty_percent * 65535)

        # Apply PWM
        self.pwm.duty_u16(pwm)

        # Optional debug
        dbg("set_power:", speed, "dir=", direction, "duty%=", duty_percent, "pwm=", pwm)

    def stop(self):
        self.pwm.duty_u16(0)
        self.in2.value(0)
