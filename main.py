from machine import UART
from pico_firmware.gpio_helper_p2 import DRV8871
from command_parser import CommandParser
import time
from watchdog import Watchdog
from led_manager import LEDStatus
from encoder import Encoder

# Motors
left_motor = DRV8871(in1_pin=14, in2_pin=15)
right_motor = DRV8871(in1_pin=16, in2_pin=17)
steering_motor = DRV8871(in1_pin=18, in2_pin=19)

# Encoders (choose pins once your holder arrives)
left_encoder = Encoder(pin_a=10, pin_b=11)
right_encoder = Encoder(pin_a=12, pin_b=13)
steering_encoder = Encoder(pin_a=20, pin_b=21)

led = LEDStatus()
uart = UART(0, baudrate=115200, tx=0, rx=1)
wd = Watchdog(led_status=led)

parser = CommandParser(
    uart,
    left_motor,
    right_motor,
    steering_motor,
    wd,
    left_encoder=left_encoder,
    right_encoder=right_encoder,
    steering_encoder=steering_encoder,
    verbose=True
)

# Simple proportional steering controller
Kp = 0.15  # tune this

while True:
    if uart.any():
        line = uart.readline()
        if line:
            parser.handle_line(line)

    # Closed-loop steering control
    if parser.steering_target is not None and parser.steering_encoder is not None:
        current = steering_encoder.get_position()
        error = parser.steering_target - current

        command = int(Kp * error)
        command = max(min(command, 100), -100)

        steering_motor.set_speed(command)

    time.sleep(0.01)
