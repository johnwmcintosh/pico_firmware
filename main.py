import firmware
from machine import UART

try:
    firmware.main()
except Exception as e:
    uart = UART(0, baudrate=115200, tx=0, rx=1)
    uart.write(b"CRASH: " + str(e).encode() + b"\r\n")
    raise
