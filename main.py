from machine import UART, Timer
from pico_firmware.gpio_helper_p2 import DRV8871
from command_parser import CommandParser
import time
from watchdog import Watchdog

left_motor = DRV8871(in1_pin=14, in2_pin=15)
right_motor = DRV8871(in1_pin=16, in2_pin=17)
steering_motor = DRV8871(in1_pin=18, in2_pin=19)

uart = UART(0, baudrate=115200, tx=0, rx=1)


wd = Watchdog()
parser = CommandParser(uart, left_motor, right_motor, steering_motor, wd, verbose=True)

while True:
    if uart.any():
        line = uart.readline()
        if line:
            parser.handle_line(line)
    time.sleep(0.01)
