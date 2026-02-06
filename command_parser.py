
import math
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

        elif cmd.startswith("STEER"):
            if len(parts) == 2:
                try:
                    target = int(parts[1])
                    self.steering_target = target
                    self.uart.write(f"OK STEER {target}\n".encode())
                except:
                    self.uart.write(b"ERR\n")

        elif cmd == "ENC_STEER?":
            if self.steering_encoder is not None:
                pos = self.steering_encoder.get_position()
                vel = self.steering_encoder.get_velocity()
                self.uart.write(f"ENC_STEER {pos} {vel}\n".encode())
            else:
                self.uart.write(b"ERR NO_STEERING_ENCODER\n")

# ---------------------------------------------------------
# OPTIONAL STEERING PID
# ---------------------------------------------------------
    def update_steering(self):
        assert self.steering_encoder is not None
        assert self.steering_target is not None

        pos = self.steering_encoder.get_position()
        vel = self.steering_encoder.get_velocity()
        error = self.steering_target - pos

        if abs(error) < 2:
            self.steering_motor.stop()
            return

        k = 0.8
        cmd = int(k * error)
        cmd = max(-100, min(100, cmd))

        self.steering_motor.set_speed(cmd)

# ---------------------------------------------------------
# ROS CMD_VEL HANDLER
# ---------------------------------------------------------
    def handle_cmd_vel(self, linear, angular):
        wheelbase = 0.25
        track = 0.18
        max_speed = 1.0
        max_steer_deg = 17.0

        if abs(angular) < 1e-6 or abs(linear) < 1e-6:
            steer_deg = 0.0
        else:
            steer_rad = math.atan((wheelbase * angular) / linear)
            steer_deg = max(min(math.degrees(steer_rad), max_steer_deg), -max_steer_deg)

        self.steering_target = steer_deg

        left_speed = linear - (angular * track / 2)
        right_speed = linear + (angular * track / 2)

        left_speed = max(min(left_speed, max_speed), -max_speed)
        right_speed = max(min(right_speed, max_speed), -max_speed)

        left_pwm = int(left_speed * 100)
        right_pwm = int(right_speed * 100)

        self.left_motor.set_speed(left_pwm)
        self.right_motor.set_speed(right_pwm)
        self.steering_motor.set_angle_deg(steer_deg)

        if self.verbose:
            print(f"[CMD] lin={linear:.2f} ang={angular:.2f} → L={left_pwm} R={right_pwm} steer={steer_deg:.1f}")

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

        uart.write(f"ODOM {left_m:.5f} {right_m:.5f} {steer_deg:.2f}\n")
