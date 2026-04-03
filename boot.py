# boot.py
# SAFETY FILE — DO NOT REMOVE
# Ensures all motor driver pins are LOW at boot so motors cannot move
# before main.py starts. Update MOTOR_PINS if hardware changes.

from machine import Pin

EXPECTED_MOTOR_PINS = [4, 5, 16, 18, 22, 28]

for p in EXPECTED_MOTOR_PINS:
    Pin(p, Pin.OUT, value=0)

for p in EXPECTED_MOTOR_PINS:
    if Pin(p).value() not in (0, 1):
        print("Warning: Motor pin", p, "not initialized correctly")
