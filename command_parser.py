
from encoder import Encoder

class CommandParser:
    def __init__(self, uart, left_motor, right_motor, steering_motor, watchdog,
        left_encoder: Encoder | None = None,
        right_encoder: Encoder | None = None,
        steering_encoder: Encoder | None = None,
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

# ---------------------------------------------------------
# MAIN LINE PARSER
# ---------------------------------------------------------
    def handle_line(self, line):
        print("HANDLE:", line)

        try:
            if isinstance(line, (bytes, bytearray)):
                line = line.decode().strip()
            else:
                line = line.strip()

            if not line:
                return

            parts = line.split()
            cmd = parts[0].upper()

        except Exception as e:
            print("Parser error:", e)
            return

        if cmd == "PYTHON":
            raise KeyboardInterrupt

        elif cmd == "CMD":
            try:
                linear = float(parts[1])
                angular = float(parts[2])
                self.handle_cmd_vel(linear, angular)
            except Exception as e:
                print("CMD parse error:", e)

        elif cmd == "PRNT":
            if parts[1].upper() == "ON":
                self.verbose = True
            else:
                self.verbose = False


    # ---------------------------------------------------------
    # OPTIONAL STEERING PID
    # ---------------------------------------------------------
    def update_steering(self):
        # Be defensive: never kill the firmware over steering
        if self.steering_encoder is None:
            return
        if self.steering_target is None:
            return

        pos = self.steering_encoder.get_position()   # counts
        error = self.steering_target - pos           # counts

        if abs(error) < 2:
            self.steering_motor.stop()
            return

        k = 0.8
        cmd = k * (error / self.steering_encoder.counts_per_lock)
        cmd = max(-1.0, min(1.0, cmd))

        self.steering_motor.set_power(cmd)

        if self.verbose:
            print("PICO steering PID: pos=", pos,
                "target=", self.steering_target,
                "error=", error,
                "cmd=", f"{cmd:.2f}")


    # ---------------------------------------------------------
    # ROS CMD_VEL HANDLER
    # ---------------------------------------------------------
    def handle_cmd_vel(self, linear, angular):
        # Clamp inputs
        throttle = max(min(linear, 1.0), -1.0)
        steering = max(min(angular, 1.0), -1.0)

        # --- DRIVE MOTOR MIXING ---
        left_cmd  = throttle - steering
        right_cmd = throttle + steering

        # Normalize if needed
        max_mag = max(abs(left_cmd), abs(right_cmd), 1.0)
        left_cmd  /= max_mag
        right_cmd /= max_mag

        self.left_motor.set_power(left_cmd)
        self.right_motor.set_power(right_cmd)

        # --- STEERING TARGET (PID) ---
        max_steer_deg = 17.0
        steer_deg = steering * max_steer_deg

        if self.steering_encoder is not None:
            try:
                target_counts = steer_deg / self.steering_encoder.deg_per_count
                self.steering_target = target_counts
            except Exception as e:
                if self.verbose:
                    print("Steering target error:", e)
                self.steering_target = None
        else:
            self.steering_target = None

        # Try to move steering, but never die
        try:
            self.update_steering()
        except Exception as e:
            if self.verbose:
                print("update_steering error:", e)

        if self.verbose:
            print(f"[CMD] throttle={throttle:.2f} steering={steering:.2f} "
                f"left={left_cmd:.2f} right={right_cmd:.2f} "
                f"steer_deg={steer_deg:.1f}")

# ---------------------------------------------------------
# ODOMETRY EMISSION
# ---------------------------------------------------------
    def emit_odometry(self, uart):
        assert self.left_encoder is not None
        assert self.right_encoder is not None
        assert self.steering_encoder is not None

        left_m = self.left_encoder.distance_m()
        right_m = self.right_encoder.distance_m()
        steer_deg = self.steering_encoder.angle_deg_clamped()

        if abs(left_m) > 0.000 or abs(right_m) > 0.000 or abs(steer_deg) > 0.00:
            uart.write(f"ODOM {left_m:.5f} {right_m:.5f} {steer_deg:.2f}\n")
