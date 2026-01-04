# led_manager.py
from machine import Pin, Timer

class LEDStatus:
    def __init__(self, pin=25):
        self.led = Pin(pin, Pin.OUT)
        self.mode = "heartbeat"
        self.timer = Timer()

        # State for watchdog blink pattern
        self._wd_step = 0

        self._start_heartbeat()

    # ---------------------------------------------------------
    # HEARTBEAT MODE (simple toggle)
    # ---------------------------------------------------------
    def _start_heartbeat(self):
        self.timer.init(
            freq=2,  # 1 Hz blink (toggle at 2 Hz)
            mode=Timer.PERIODIC,
            callback=self._heartbeat_cb
        )

    def _heartbeat_cb(self, t):
        if self.mode == "heartbeat":
            self.led.toggle()

    # ---------------------------------------------------------
    # WATCHDOG MODE (non-blocking triple-blink pattern)
    # ---------------------------------------------------------
    def _watchdog_cb(self, t):
        if self.mode != "watchdog":
            return

        # Pattern timing (in steps):
        # 0: ON
        # 1: OFF
        # 2: ON
        # 3: OFF
        # 4: ON
        # 5: OFF
        # 6: pause
        if self._wd_step == 0:
            self.led.on()
        elif self._wd_step == 1:
            self.led.off()
        elif self._wd_step == 2:
            self.led.on()
        elif self._wd_step == 3:
            self.led.off()
        elif self._wd_step == 4:
            self.led.on()
        elif self._wd_step == 5:
            self.led.off()
        elif self._wd_step == 6:
            # pause step — LED stays off
            pass

        # Advance step
        self._wd_step = (self._wd_step + 1) % 7

    def set_heartbeat(self):
        if self.mode != "heartbeat":
            self.mode = "heartbeat"
            self.timer.deinit()
            self._start_heartbeat()

    def set_watchdog(self):
        if self.mode != "watchdog":
            self.mode = "watchdog"
            self._wd_step = 0
            self.timer.deinit()
            # 80ms per step → matches your original timing
            self.timer.init(
                period=80,
                mode=Timer.PERIODIC,
                callback=self._watchdog_cb
            )
