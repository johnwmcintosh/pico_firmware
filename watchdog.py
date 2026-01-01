from machine import Timer

class Watchdog:
    def __init__(self, timer_id=0, timeout_ms=2000, callback=None):
        self.timer = Timer(timer_id)
        self.timeout_ms = timeout_ms
        self.callback = callback or self._default_callback
        self._armed = False

    def _default_callback(self, t):
        print("Watchdog timeout – no callback provided")

    def start(self):
        self.timer.init(period=self.timeout_ms, mode=Timer.PERIODIC, callback=self.callback)
        self._armed = True

    def reset(self):
        if self._armed:
            self.timer.deinit()
            self.start()

    def stop(self):
        self.timer.deinit()
        self._armed = False
