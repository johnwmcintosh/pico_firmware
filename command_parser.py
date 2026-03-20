from encoder import SteeringEncoder, DrivingEncoder

class CommandParser:
    def __init__(self, uart, left_motor, right_motor, steering_motor, watchdog,
                 left_encoder: DrivingEncoder | None = None,
                 right_encoder: DrivingEncoder | None = None,
                 steering_encoder: SteeringEncoder | None = None,
                 steering_target: float | None = None,
                 verbose=True):

        self.uart = uart
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.steering_motor = steering_motor
        self.watchdog = watchdog

        self.left_encoder = left_encoder
        self.right_encoder = right_encoder
        self.steering_encoder = steering_encoder

        # normalized steering target (-1.0 .. +1.0)
        self.steering_target = steering_target

        self.verbose = verbose

    # ---------------------------------------------------------
    # MAIN LINE PARSER
    # ---------------------------------------------------------
    def handle_line(self, line):
        #print("HANDLE:", line)
        print("RUNTIME STEER MOTOR:", self.steering_motor.pin_in1, self.steering_motor.pin_in2)

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
                self.update_driving_stick(linear)
                self.update_steering_stick(angular)
            except Exception as e:
                print("CMD parse error:", e)

        elif cmd == "PRNT":
            self.verbose = (parts[1].upper() == "ON")

    # ---------------------------------------------------------
    # STEERING PID (normalized)
    # ---------------------------------------------------------
    def update_steering_stick(self, x):
        # x in [-1, 1]
        DEAD = 0.05
        p_min = 0.60   # minimum effective PWM (motor starts moving)
        p_max = 0.80   # safe max for rack

        ax = abs(x)

        # Deadband
        if ax < DEAD:
            self.steering_motor.stop()
            return
        
        pwr = 1 - ax

        # Map |x| from [DEAD, 1] -> [p_min, p_max]
        t = (pwr - DEAD) / (1.0 - DEAD)
        power = p_min + t * (p_max - p_min)

        # Restore direction
        if x < 0:
            power = -power

        print("x:", x, "steering power:", power)
        self.steering_motor.set_power(power)

    def update_steering_pid(self):
        if self.steering_encoder is None:
            return
        if self.steering_target is None:
            return

        current = self.steering_encoder.get_angle()   # -1..1
        error = self.steering_target - current

        # ---------------------------------------------------------
        # DEADZONE: prevent jitter when nearly correct
        # ---------------------------------------------------------
        if abs(error) < 0.03:
            self.steering_motor.stop()
            return

        # ---------------------------------------------------------
        # PROPORTIONAL CONTROL
        # ---------------------------------------------------------
        k = 1.0
        cmd = k * error

        # ---------------------------------------------------------
        # OUTPUT SHAPING: soften small corrections
        # ---------------------------------------------------------
        if abs(cmd) < 0.25:
            cmd *= 0.5

        # ---------------------------------------------------------
        # RATE LIMITING: prevents sudden jerks
        # ---------------------------------------------------------
        max_step = 0.12
        cmd = max(min(cmd, max_step), -max_step)

        # ---------------------------------------------------------
        # SAFETY: never push past mechanical limits
        # ---------------------------------------------------------
        if abs(current) >= 1.0 and (error * current) > 0:
            self.steering_motor.stop()
            return

        # ---------------------------------------------------------
        # FINAL CLAMP
        # ---------------------------------------------------------
        cmd = max(-1.0, min(1.0, cmd))

        self.steering_motor.set_power(cmd)

        if self.verbose:
            print("STEER PID:",
                "current=", f"{current:.2f}",
                "target=", f"{self.steering_target:.2f}",
                "error=", f"{error:.2f}",
                "cmd=", f"{cmd:.2f}")

    # ---------------------------------------------------------
    # ROS CMD_VEL HANDLER
    # ---------------------------------------------------------
    def update_driving_stick(self, linear):
        # x in [-1, 1]
        DEAD = 0.00
        p_min = 0.01   # minimum effective PWM (motor starts moving)
        p_max = 0.99   # safe max for rack
    
        x = linear
        ax = abs(x)

        # Deadband
        if ax < .01:
            self.left_motor.stop()
            self.right_motor.stop()
            return
        
        pwr = 1 - ax

        # Map |x| from [DEAD, 1] -> [p_min, p_max]
        t = (pwr - DEAD) / (1.0 - DEAD)
        power = p_min + t * (p_max - p_min)

        # Restore direction
        if x > 0:
            power = -power

        print("x:", x, "drive power:", power)
        self.left_motor.set_power(power)
        self.right_motor.set_power(power)
    

    def handle_cmd_vel_pid(self, linear, angular):
        # Clamp inputs
        throttle = max(min(linear, 1.0), -1.0)
        steering = max(min(angular, 1.0), -1.0)

        # --- DRIVE MOTOR MIXING ---
        left_cmd  = throttle
        right_cmd = throttle

        # Normalize if needed
        max_mag = max(abs(left_cmd), abs(right_cmd), 1.0)
        left_cmd  /= max_mag
        right_cmd /= max_mag

        self.left_motor.set_power(left_cmd)
        self.right_motor.set_power(right_cmd)

        # --- STEERING TARGET (normalized) ---
        self.steering_target = -steering  # invert if needed
        
        # steering input from ROS/gamepad is already -1..1
        desired = -steering   # invert if needed

        # SNAP-TO-CENTER when stick released
        if abs(desired) < 0.05:
            desired = 0.0

        # SMOOTH TARGET: prevents instant jumps
        alpha = 0.25
        self.steering_target = (
            self.steering_target * (1 - alpha) + desired * alpha
        )

        print(
            "steer_norm: ",
            self.steering_target
        )

        # Try to move steering, but never die
        try:
            self.update_steering_pid()
        except Exception as e:
            if self.verbose:
                print("update_steering error:", e)

        if self.verbose:
            print(
                "[CMD] throttle= ", throttle," steering= ",steering,
                "left= ",left_cmd," right= ",right_cmd,
                " steer_target= ",self.steering_target
            )

    # ---------------------------------------------------------
    # ODOMETRY EMISSION
    # ---------------------------------------------------------
    def emit_odometry(self, uart):
        assert self.left_encoder is not None
        assert self.right_encoder is not None
        assert self.steering_encoder is not None

        left_m = self.left_encoder.distance_m()
        right_m = self.right_encoder.distance_m()
        steer_angle = self.steering_encoder.get_angle()  # normalized

        uart.write(
            f"ODOM {left_m:.5f} {right_m:.5f} {steer_angle:.3f}\r\n"
        )
