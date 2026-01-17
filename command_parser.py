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

        if cmd == "PYTHON":
            raise KeyboardInterrupt

        # Closed-loop steering command
        elif cmd.startswith("STEER "):
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

    def update_steering(self):
        pos, vel = self.steering_encoder.get_state()
        error = self.steering_target - pos

        # deadzone
        if abs(error) < 2:
            self.steering_motor.stop()
            return

        # proportional gain
        k = 0.8  # tune this later
        cmd = int(k * error)

        # clamp
        cmd = max(-100, min(100, cmd))

        self.steering_motor.set_speed(cmd)