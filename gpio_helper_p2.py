from machine import Pin, PWM

class DRV8871:
    def __init__(self, pin_dir, pin_en):
        # pin_en -> IN1 (PWM)
        # pin_dir -> IN2 (direction)
        self.in2 = Pin(pin_dir, Pin.OUT)     # Direction
        self.pwm = PWM(Pin(pin_en))          # Speed via PWM on IN1
        self.pwm.freq(20000)                 # 20 kHz is a good quiet value

        self.MIN_DUTY = 59
        self.MAX_DUTY = 100

        self.stop()

    def set_speed(self, speed):
        # Clamp logical speed
        speed = max(min(speed, 100), -100)

        if speed == 0:
            print("set_speed: 0 → stop")
            self.stop()
            return

        # Direction + debug label
        if speed > 0:
            self.in2.value(0)
            direction = "FWD"
        else:
            self.in2.value(1)
            direction = "REV"

        # Map logical magnitude (0–100) into physical duty range (MIN_DUTY–MAX_DUTY)
        mag = abs(speed) / 100.0
        duty_percent = self.MIN_DUTY + mag * (self.MAX_DUTY - self.MIN_DUTY)
        duty_u16 = int((duty_percent / 100.0) * 65535)

        print("set_speed:", speed,
              "dir=", direction,
              "duty%=", duty_percent,
              "duty_u16=", duty_u16)

        self.pwm.duty_u16(duty_u16)

    def stop(self):
        self.pwm.duty_u16(0)
        self.in2.value(0)
