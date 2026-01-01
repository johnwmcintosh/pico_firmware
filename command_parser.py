
class CommandParser:
    def __init__(self, uart, left_motor, right_motor, steering_motor, watchdog, verbose=False):
        self.uart = uart
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.steering_motor = steering_motor
        self.watchdog = watchdog
        self.verbose = verbose

    def log(self, msg):
        if self.verbose:
            self.uart.write(f"[LOG] {msg}\n")

    def handle_line(self, line):
        try:
            cmd = line.strip().decode()
            self.log(f"Received: {cmd}")

            if cmd == "HB":
                self.watchdog.reset()
                self.uart.write(b"OK HB\n")

            elif cmd.startswith("STEER:"):
                angle = float(cmd[6:])
                self.steering_motor.set_speed(angle)
                self.uart.write(f"OK STEER {angle:.2f}\n".encode())

            elif cmd.startswith("FWD:"):
                duty = float(cmd[4:])
                self.left_motor.forward(int(duty * 65535))
                self.right_motor.forward(int(duty * 65535))
                self.uart.write(f"OK FWD {duty:.2f}\n".encode())

            elif cmd.startswith("REV:"):
                duty = float(cmd[4:])
                self.left_motor.reverse(int(duty * 65535))
                self.right_motor.reverse(int(duty * 65535))
                self.uart.write(f"OK REV {duty:.2f}\n".encode())

            elif cmd == "STOP":
                self.left_motor.stop()
                self.right_motor.stop()
                self.steering_motor.stop()
                self.uart.write(b"OK STOPPED\n")

            elif cmd == "PING":
                self.uart.write(b"PONG\n")

            elif cmd == "STATUS":
                self.uart.write(b"STATUS: TODO\n")  # You can expand this

            else:
                self.uart.write(b"ERR UNKNOWN CMD\n")

        except Exception as e:
            self.uart.write(f"ERR {str(e)}\n".encode())
