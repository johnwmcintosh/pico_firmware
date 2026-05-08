from machine import Pin, PWM


class DRV8871:
    def __init__(self, pin_in1: int, pin_in2: int):
        # Store pin numbers
        self.pin_in1 = pin_in1
        self.pin_in2 = pin_in2

        # Create output pins
        self.in1 = Pin(pin_in1, Pin.OUT)
        self.in2 = Pin(pin_in2, Pin.OUT)

        # Create PWM object for typing; immediately disable it
        self.pwm = PWM(self.in1)
        self.pwm.deinit()
        self.pwm_active = False
        self.pwm_pin_num = None

        # Immediately enter safe state
        self._release_pwm()
        self.in1.low()
        self.in2.low()

    # ------------------------------------------------------------
    def _release_pwm(self) -> None:
        """Stop PWM and return the pin to a LOW output state."""
        if self.pwm_active:
            self.pwm.deinit()
            self.pwm_active = False

        if self.pwm_pin_num is not None:
            Pin(self.pwm_pin_num, Pin.OUT).low()
            self.pwm_pin_num = None

    # ------------------------------------------------------------
    def _attach_pwm(self, pin_num: int) -> None:
        """Attach PWM to the given pin, ensuring it starts LOW."""
        self._release_pwm()

        # Ensure pin is LOW before PWM takes over
        Pin(pin_num, Pin.OUT).low()

        pwm_pin = Pin(pin_num, Pin.OUT)
        self.pwm = PWM(pwm_pin)
        self.pwm.freq(20000)
        self.pwm_active = True
        self.pwm_pin_num = pin_num

    # ------------------------------------------------------------
    def stop(self) -> None:
        """Brake mode: both pins HIGH."""
        self._release_pwm()
        self.in1.low()
        self.in2.low()

    def coast(self):
        # Coast mode
        self._release_pwm()
        self.in1.low()
        self.in2.low()

    # ------------------------------------------------------------
    # Compatibility with old parser API
    def set_raw_power(self, value: float) -> None:
        self.set_power(value)

    # ------------------------------------------------------------
    def set_power(self, power: float) -> None:
        """Set motor power in range [-1.0, 1.0]."""
        power = max(min(power, 1.0), -1.0)

        if power == 0:
            self.coast()
            return

        duty = int(abs(power) * 65535)

        if power > 0:
            # Forward: IN1 = HIGH, PWM on IN2
            Pin(self.pin_in1, Pin.OUT).high()
            Pin(self.pin_in2, Pin.OUT).low()
            self._attach_pwm(self.pin_in2)
            self.pwm.duty_u16(duty)

        else:
            # Reverse: IN2 = HIGH, PWM on IN1
            Pin(self.pin_in2, Pin.OUT).high()
            Pin(self.pin_in1, Pin.OUT).low()
            self._attach_pwm(self.pin_in1)
            self.pwm.duty_u16(duty)
