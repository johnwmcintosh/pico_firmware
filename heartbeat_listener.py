from machine import UART, Timer
import time

uart = UART(0, baudrate=115200, tx=0, rx=1)

# Track last heartbeat time
last_hb = time.ticks_ms()
TIMEOUT_MS = 2000

timeout_flag = False

def watchdog_check(timer):
    global timeout_flag, last_hb
    if time.ticks_diff(time.ticks_ms(), last_hb) > TIMEOUT_MS:
        timeout_flag = True

# Run watchdog check every 200 ms
watchdog = Timer(0)
watchdog.init(period=200, mode=Timer.PERIODIC, callback=watchdog_check)

while True:
    if uart.any():
        msg = uart.readline()
        if msg and msg.strip() == b'HB':
            last_hb = time.ticks_ms()
            timeout_flag = False

    if timeout_flag:
        print("Watchdog timeout – no heartbeat received")
        timeout_flag = False  # prevent repeated prints

    time.sleep(0.05)
