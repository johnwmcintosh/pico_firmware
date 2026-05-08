import firmware
from machine import UART
from machine import Pin
import time

time.sleep_ms(2000)  # wait 2 seconds for USB enumeration to settle

MOTOR_PINS = [4, 5, 7, 16, 18, 22]  # updated: 28 removed, 7 added

for p in MOTOR_PINS:
    Pin(p, Pin.OUT, value=0, pull=None)

# Verify pins are actually LOW
for p in MOTOR_PINS:
    val = Pin(p).value()
    if val != 0:
        print(f"WARNING: GP{p} is HIGH after init — E9 errata latch suspected")
    else:
        print(f"GP{p}: OK")

try:
    firmware.main()
except Exception as e:
    uart = UART(0, baudrate=115200, tx=0, rx=1)
    uart.write(b"CRASH: " + str(e).encode() + b"\r\n")
    raise
