# firmware.py

import time
import asyncio
import _thread
from machine import UART

from led_manager import LEDStatus, startup_blink, enter_error_mode
from watchdog import Watchdog

from encoder import Encoder
from gpio_helper_p2 import DRV8871
from command_parser import CommandParser

import uasyncio as asyncio

class ModeBlinker:
    def __init__(self, led, mode: str):
        self.led = led
        self.mode = mode
        asyncio.create_task(self._loop())

    async def _loop(self):
        while True:
            if self.mode == "RUN":
                self.led.on()
                await asyncio.sleep(0.1)
                self.led.off()
                await asyncio.sleep(0.1)
                self.led.on()
                await asyncio.sleep(0.1)
                self.led.off()
                await asyncio.sleep(0.7)
            else:
                self.led.on()
                await asyncio.sleep(0.2)
                self.led.off()
                await asyncio.sleep(1.8)


def init_uart_for_run_mode():
    return UART(0, baudrate=115200, tx=0, rx=1)


def init_motors():
    steer_motor = DRV8871(pin_dir=16, pin_en=19)
    drive_left  = DRV8871(pin_dir=5,  pin_en=4)
    drive_right = DRV8871(pin_dir=22, pin_en=28)
    return steer_motor, drive_left, drive_right


def init_encoders():
    steer_encoder = Encoder(pin_a=6, pin_b=7, counts_per_lock=56, deg_per_lock=17.0)
    drive_left_encoder = Encoder(pin_a=8, pin_b=9)
    drive_right_encoder = Encoder(pin_a=10, pin_b=11)
    return steer_encoder, drive_left_encoder, drive_right_encoder


def init_led_and_watchdog():
    led = LEDStatus()
    watchdog = Watchdog(timeout_ms=2000)
    return led, watchdog


def run_mode_loop(uart, parser, watchdog, led):
    last_odom_ms = time.ticks_ms()
    last_heartbeat = time.ticks_ms()

    while True:
        try:
            # --- UART INPUT ---
            print(print(time.ticks_ms())
)
            print("A")
            line = uart.readline()
            if line:
                try:
                    decoded = line.decode().strip()
                except UnicodeError:
                    decoded = ""

                if decoded.startswith("CMD"):
                    parser.handle_line(decoded)

            # --- STEERING ---
            print("B")
            if parser.steering_target is not None:
                parser.update_steering()

            # --- ODOMETRY ---
            print("C")
            now = time.ticks_ms()
            if time.ticks_diff(now, last_odom_ms) > 50:
                parser.emit_odometry(uart)
                last_odom_ms = now

            # --- HOUSEKEEPING ---
            print("D")
            watchdog.reset()
            watchdog.check()
            led.update()

            # --- HEARTBEAT @ 1 Hz ---
            print("E")
            if time.ticks_diff(now, last_heartbeat) > 1000:
                uart.write(b"HEARTBEAT\n")
                last_heartbeat = now

           

        except Exception as e:
            uart.write(b"RUN loop error\n")
            uart.write(str(e).encode() + b"\n")
            enter_error_mode(led)
            time.sleep_ms(50)


def main():
    # Initialize UART0 immediately so we always have visibility
    uart = init_uart_for_run_mode()
    uart.write(b"MAIN: entered main()\n")

    # Init LED + watchdog
    led, watchdog = init_led_and_watchdog()
    uart.write(b"MAIN: init_led_and_watchdog\n")

    # Simple RUN-mode startup blink
    startup_blink(led, "RUN")
    uart.write(b"MAIN: startup_blink\n")

    # Start mode blinker in RUN pattern
    ModeBlinker(led, "RUN")
    uart.write(b"MAIN: mode_blinker\n")

    # Hardware init
    uart.write(b"MAIN: init_motors\n")
    try:
        steer_motor, drive_left, drive_right = init_motors()
    except Exception as e:
        uart.write(b"ERR: init_motors failed\n")
        enter_error_mode(led)
        return

    uart.write(b"MAIN: init_encoders\n")
    try:
        steer_encoder, drive_left_encoder, drive_right_encoder = init_encoders()
    except Exception as e:
        uart.write(b"ERR: init_encoders failed\n")
        enter_error_mode(led)
        return

    uart.write(b"MAIN: zero steer encoder\n")
    try:
        steer_encoder.zero()
    except Exception as e:
        uart.write(b"ERR: steer_encoder.zero failed\n")
        enter_error_mode(led)
        return

    # Start watchdog
    watchdog.start()
    uart.write(b"MAIN: watchdog started\n")

    # Parser init
    uart.write(b"MAIN: init parser\n")
    try:
        parser = CommandParser(
            uart=uart,
            left_motor=drive_left,
            right_motor=drive_right,
            steering_motor=steer_motor,
            watchdog=watchdog,
            left_encoder=drive_left_encoder,
            right_encoder=drive_right_encoder,
            steering_encoder=steer_encoder,
            verbose=True,
        )
    except Exception as e:
        uart.write(b"ERR: parser init failed\n")
        enter_error_mode(led)
        return

    uart.write(b"MAIN: entering run loop\n")
    run_mode_loop(uart, parser, watchdog, led)
