import time
import uasyncio as asyncio
from machine import UART, Pin, I2C

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


def init_display():
    import ssd1306  # type: ignore
    i2c = I2C(0, sda=Pin(20), scl=Pin(21), freq=400000)
    return ssd1306.SSD1306_I2C(128, 32, i2c)


def main():
    uart = init_uart_for_run_mode()
    print("MAIN: entered main()\r\n")

    try:
        display = init_display()
    except Exception as e:
        print("DISPLAY init failed:", e)
        display = None

    if (display is not None):
        display.fill(0)
        display.text("Booting...", 0, 0, 1)
        display.show()
        
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
    last_odom = time.ticks_ms()
    last_display = time.ticks_ms()
    last_awaiting_ros = time.ticks_ms()
    TIMEOUT_MS = 5000
    ODOM_INTERVAL_MS = 100   # 10Hz
    DISPLAY_INTERVAL_MS = 2000
    AWAITING_ROS_MS = 2000
    
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

        # Display update
        if display and time.ticks_diff(time.ticks_ms(), last_display) >= DISPLAY_INTERVAL_MS:
            try:
                #if parser.lidar_latch > 0:
                #    parser.lidar_latch -= 1
                dist_str = f"{parser.lidar_dist:.1f}" if parser.lidar_dist is not None else "--"
                angle_str = f"{parser.lidar_angle:.1f}" if parser.lidar_angle is not None else "--"
                display.fill(0)
                display.text(f"{parser.lidar_status}:", 0, 0, 1)
                display.text(f"Dist:{dist_str}m", 0, 8, 1)
                display.text(f"Ang:{angle_str}d", 0, 16, 1)

                display.show()
            except Exception as e:
                print("DISPLAY error:", e)
            last_display = time.ticks_ms()

        # Steering PID, odometry, etc
        if time.ticks_diff(time.ticks_ms(), last_odom) >= ODOM_INTERVAL_MS:
            try:
                parser.emit_odometry(uart)
            except Exception as e:
                print("ODOM error:", e)
            last_odom = time.ticks_ms()

        time.sleep_ms(10)
