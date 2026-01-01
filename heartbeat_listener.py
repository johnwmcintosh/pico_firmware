from machine import UART, Timer
from pico_firmware.gpio_helper_p2 import DRV8871
import time

motor = DRV8871(in1_pin=14, in2_pin=15)
uart = UART(0, baudrate=115200, tx=0, rx=1)  # Adjust pins if needed

def on_watchdog(timer):
    print("Watchdog timeout – stopping motor")
    motor.stop()

watchdog = Timer(0)
watchdog.init(period=2000, mode=Timer.PERIODIC, callback=on_watchdog)

def reset_watchdog():
    watchdog.deinit()
    watchdog.init(period=2000, mode=Timer.PERIODIC, callback=on_watchdog)

while True:
    if uart.any():
        msg = uart.readline()
        if msg and msg.strip() == b'HB':
            reset_watchdog()
    time.sleep(0.05)
