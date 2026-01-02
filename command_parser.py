class CommandParser:
    def __init__(self, uart, left_motor, right_motor, steering_motor, watchdog,
                 left_encoder=None, right_encoder=None, steering_encoder=None,
                 verbose=False):

        self.uart = uart
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.steering_motor = steering_motor
        self.watchdog = watchdog

        self.left_encoder = left_encoder
        self.right_encoder = right_encoder
        self.steering_encoder = steering_encoder

        self.steering_target = None
        self.verbose = verbose

    def handle_line(self, line):
        try:
            cmd = line.decode().strip()
        except:
            return

        # Closed-loop steering command
        if cmd.startswith("STEER "):
            parts = cmd.split()
            if len(parts) == 2:
                try:
                    target = int(parts[1])
                    self.steering_target = target
                    self.uart.write(f"OK STEER {target}\n".encode())
                except:
                    self.uart.write(b"ERR\n")

        # Encoder queries
        elif cmd == "ENC_STEER?":
            if self.steering_encoder is not None:
                pos = self.steering_encoder.get_position()
                vel = self.steering_encoder.get_velocity()
                self.uart.write(f"ENC_STEER {pos} {vel}\n".encode())
            else:
                self.uart.write(b"ERR NO_STEERING_ENCODER\n")


