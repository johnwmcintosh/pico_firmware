import time
import uasyncio as asyncio
from machine import UART, Pin

from led_manager import LEDStatus, startup_blink, enter_error_mode
from watchdog import Watchdog

from encoder import DrivingEncoder, SteeringEncoder
from gpio_helper_p2 import DRV8871
from command_parser import CommandParser


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
    return UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))


def init_motors():
    steer_motor = DRV8871(pin_in1=16, pin_in2=18)
    drive_left  = DRV8871(pin_in1=5,  pin_in2=4)
    drive_right = DRV8871(pin_in1=22, pin_in2=7)
    return steer_motor, drive_left, drive_right


def init_encoders():
    steer_encoder = SteeringEncoder(pin_a=26, pin_b=27)
    drive_left_encoder = DrivingEncoder(pin_a=8, pin_b=9)
    drive_right_encoder = DrivingEncoder(pin_a=10, pin_b=11)
    return steer_encoder, drive_left_encoder, drive_right_encoder


def init_led_and_watchdog():
    led = LEDStatus()
    watchdog = Watchdog(timeout_ms=2000)
    return led, watchdog


def main():
    uart = init_uart_for_run_mode()
    print("MAIN: entered main()\r\n")

    led, watchdog = init_led_and_watchdog()
    startup_blink(led, "RUN")
    ModeBlinker(led, "RUN")

    # Init motors
    print("MAIN: init_motors\r\n")
    steer_motor, drive_left, drive_right = init_motors()

    # Init encoders
    print("MAIN: init_encoders\r\n")
    steer_encoder, drive_left_encoder, drive_right_encoder = init_encoders()

    # ---------------------------------------------------------
    # SMART AUTO-ZERO STEERING (TIMED, SAFE)
    # ---------------------------------------------------------
    drive_left.coast()
    drive_right.coast()
    steer_motor.coast()
    time.sleep_ms(200)

    initial_pos = steer_encoder.get_position()
    steering_target = 0.0

    # Start watchdog
    watchdog.start()

    # Init parser
    parser = CommandParser(
        uart=uart,
        left_motor=drive_left,
        right_motor=drive_right,
        steering_motor=steer_motor,
        watchdog=watchdog,
        left_encoder=drive_left_encoder,
        right_encoder=drive_right_encoder,
        steering_encoder=steer_encoder,
        steering_target=steering_target,
        verbose=True,
    )

    print("MAIN: entering run loop")

    # ---------------------------------------------------------
    # INTEGRATED UART + HEARTBEAT + WATCHDOG LOOP
    # ---------------------------------------------------------

    rx_buffer = ""
    last_hb = time.ticks_ms()
    TIMEOUT_MS = 2000

    while True:
        # -----------------------------------------
        # UART READ (non-blocking, buffered)
        # -----------------------------------------
        data = uart.read()
        if data:
            try:
                rx_buffer += data.decode()
            except UnicodeError:
                pass

            # Process complete lines
            while "\n" in rx_buffer:
                line, rx_buffer = rx_buffer.split("\n", 1)
                line = line.strip()

                if not line:
                    continue

                # -----------------------------------------
                # HEARTBEAT
                # -----------------------------------------
                if line == "HB":
                    last_hb = time.ticks_ms()
                    continue

                if line.startswith("CMD"):
                    last_hb = time.ticks_ms()

                # -----------------------------------------
                # COMMAND
                # -----------------------------------------
                #print("RX:", line)
                try:
                    parser.handle_line(line)
                except Exception as e:
                    print("CMD parse error:", e)

        # -----------------------------------------
        # WATCHDOG TIMEOUT
        # -----------------------------------------
        if time.ticks_diff(time.ticks_ms(), last_hb) > TIMEOUT_MS:
            print("WATCHDOG TIMEOUT — stopping motors")
            steer_motor.coast()
            drive_left.coast()
            drive_right.coast()
            last_hb = time.ticks_ms()  # prevent repeated prints

        # -----------------------------------------
        # LED + WATCHDOG + CONTROL LOOP
        # -----------------------------------------
        watchdog.reset()
        led.update()

        # (Optional: steering PID, odometry, etc.)

        time.sleep_ms(10)
