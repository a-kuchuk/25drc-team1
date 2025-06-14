import math

class SteeringController:
    def __init__(self, wheelbase_mm=220, track_width_mm=151,
                 max_pwm=67.3, min_pwm=34.0, servo_center_pwm=50.7):
        self.L = wheelbase_mm
        self.w = track_width_mm
        self.servo_pwm_min = min_pwm
        self.servo_pwm_max = max_pwm
        self.servo_pwm_mid = servo_center_pwm

        # Max steering angle from Ackermann geometry (inner wheel turn)
        R_outer = 500
        R_inner = R_outer - (self.w / 2)
        self.max_steering_angle_deg = math.degrees(math.atan(self.L / R_inner))  # ~27.6°

    def angle_to_pwm(self, angle_deg):
        """
        Convert a desired steering angle (±27.6°) into a PWM duty cycle (5% to 10%)
        """
        # Clamp angle to physical limits
        angle_deg = max(-self.max_steering_angle_deg, min(self.max_steering_angle_deg, angle_deg))

        # Map angle [-max, max] → PWM [min, max]
        pwm_range = self.servo_pwm_max - self.servo_pwm_min
        pwm = self.servo_pwm_mid + (angle_deg / (2 * self.max_steering_angle_deg)) * pwm_range * 2

        return max(self.servo_pwm_min, min(self.servo_pwm_max, pwm))

    def curvature_to_angle(self, curvature):
        """
        Convert a curvature (1/R) into steering angle using Ackermann geometry
        """
        if curvature == 0:
            return 0.0
        radius = 1.0 / curvature
        angle_rad = math.atan(self.L / radius)
        return math.degrees(angle_rad)