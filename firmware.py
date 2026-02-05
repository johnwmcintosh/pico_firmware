# firmware.py

import sys
import time
import _thread
import select

from machine import UART

from encoder import Encoder
from gpio_helper_p2 import DRV8871
from led_manager import LEDStatus
from watchdog import Watchdog
from command_parser import CommandParser

print(">>> firmware.py STARTED <<<")
time.sleep(0.3)

# ---------------------------------------------------------
# USB / mode detection
# ---------------------------------------------------------

def is_usb_connected():
    """
    Returns True if a USB REPL (Thonny/terminal) is active.
    Does NOT mean "Pico is powered by USB".
    """
    cls = sys.stdin.__class__.__name__
    return cls in ("USB_VCP", "TextIOWrapper")


# ---------------------------------------------------------
# Debug console (RUN MODE only, NEVER under Thonny)
# ---------------------------------------------------------

class DebugConsole:
    """
    Non-blocking stdin reader intended ONLY for standalone RUN MODE.
    NEVER run this when a USB REPL (Thonny) is attached on Pico 2.
    """

    def __init__(self, parser: CommandParser):
        self._parser = parser
        self._running = True
        _thread.start_new_thread(self._poll_loop, ())

    def _poll_loop(self):
        buf = b""
        while self._running:
            try:
                ch = sys.stdin.read(1)
                if ch:
                    if isinstance(ch, str):
                        ch = ch.encode()
                    buf += ch
                    if ch in (b"\n", b"\r"):
                        self._parser.handle_line(buf)
                        buf = b""
                time.sleep(0.01)
            except Exception as e:
                print("DEBUG poll error:", e)
                time.sleep(0.05)

    def stop(self):
        self._running = False


# ---------------------------------------------------------
# REPL writer for debug mode
# ---------------------------------------------------------

class REPLWriter:
    """
    Simple wrapper that lets the parser "write to UART" by printing to REPL.
    """

    def write(self, data):
        try:
            if isinstance(data, (bytes, bytearray, memoryview)):
                print(bytes(data).decode().rstrip())
            else:
                print(str(data))
        except Exception as e:
            print("REPLWriter error:", e)
            print(data)


# ---------------------------------------------------------
# Hardware initialization
# ---------------------------------------------------------

def init_uart_for_run_mode():
    """
    Initialize UART0 for robot RUN MODE.
    Adjust pins/baudrate if needed.
    """
    return UART(0, baudrate=115200, tx=0, rx=1)


def init_motors():
    """
    Initialize DRV8871 motor drivers.
    Adjust pin numbers to match your wiring.
    """
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
    """
    Initialize status LED and watchdog.
    """
    led = LEDStatus(pin=25)
    wd = Watchdog(led_status=led)
    return led, wd

# ---------------------------------------------------------
# RUN MODE loop
# ---------------------------------------------------------

def run_mode_loop(uart, parser: CommandParser, watchdog: Watchdog):
    print("RUN MODE: Entering main loop.")
    while True:
        try:
            print("RUN LOOP ACTIVE — NEW FIRMWARE")

            # Handle incoming UART commands
            if uart.any():
                line = uart.readline()
                if line:
                    parser.handle_line(line)
                
                parser.emit_odometry(uart)

        except Exception as e:
            print("RUN loop error:", e)
            time.sleep(0.05)

# ---------------------------------------------------------
# System status helper
# ---------------------------------------------------------

def system_status(parser=None, debug_console_running=False):
    """
    Print a snapshot of system state:
      - USB / REPL status
      - DebugConsole status
      - Parser + motors + encoders
    Safe to call anytime from REPL or code.
    """
    print("----- SYSTEM STATUS -----")

    cls = sys.stdin.__class__.__name__
    print("stdin class =", cls)
    if cls in ("USB_VCP", "TextIOWrapper"):
        print("USB_CONNECTED = True  (USB REPL active)")
    else:
        print("USB_CONNECTED = False (no USB REPL attached)")

    print("DebugConsole running =", debug_console_running)

    if parser is None:
        print("Parser = None")
    else:
        print("Parser = OK")
        print("  Left motor:", parser.left_motor)
        print("  Right motor:", parser.right_motor)
        print("  Steering motor:", parser.steering_motor)
        print("  Left encoder:", parser.left_encoder)
        print("  Right encoder:", parser.right_encoder)
        print("  Steering encoder:", parser.steering_encoder)
        print("  Verbose =", parser.verbose)

    print("-------------------------")


# ---------------------------------------------------------
# Entry point
# ---------------------------------------------------------

def main(enable_debug_console=False):
    """
    Main entry point for firmware.

    Returns:
        (parser, debug_console_running)
    so you can introspect from REPL.
    """
    USB_CONNECTED = is_usb_connected()
    print("USB_CONNECTED =", USB_CONNECTED)
    print("stdin class =", sys.stdin.__class__.__name__)

    debug_console_running = False

    # Hardware setup
    led, watchdog = init_led_and_watchdog()
    steer_motor, drive_left, drive_right = init_motors()
    steer_encoder, drive_left_encoder, drive_right_encoder = init_encoders()
    
    steer_encoder.zero()
    
    # Mode-specific UART
    if USB_CONNECTED:
        uart = REPLWriter()
    else:
        uart = init_uart_for_run_mode()

    # Create parser and inject dependencies
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

    # DEBUG MODE (USB REPL / Thonny)
    if USB_CONNECTED:
        print("DEBUG MODE: USB detected, REPL input active.")
        print("DebugConsole DISABLED in USB mode.")
        print("REPL ready. Type commands like ENC_STEER? directly.")
        # No loops here; just return control to REPL
        return parser, debug_console_running

    # RUN MODE (standalone robot)
    print("RUN MODE: Starting main loop.")
    if enable_debug_console:
        print("Starting DebugConsole (RUN MODE only)...")
        DebugConsole(parser)
        debug_console_running = True

    # This will not return in normal RUN MODE
    run_mode_loop(uart, parser, watchdog)
    # If you ever add an exit condition, you could return here:
    # return parser, debug_console_running


# ---------------------------------------------------------
# No auto-run on import
# ---------------------------------------------------------
# IMPORTANT:
# Do NOT call main() here. You must run it manually from Thonny:
#   import firmware
#   parser, dc = firmware.main()
#
# This keeps boot safe and avoids "device busy" traps.
