# boot.py
# SAFETY FILE — DO NOT REMOVE
# Ensures all motor driver pins are LOW at boot so motors cannot move
# before main.py starts. Update MOTOR_PINS if hardware changes.
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