import math
import pigpio


GPIO_PIN = 17               # GPIO pin connected to servo
WHEELBASE_MM = 220
TRACK_WIDTH_MM = 151
SERVO_RANGE_DEG = 50        # ±50° from datasheet
PULSE_CENTER_US = 1520      # Neutral pulse width (μs)
PULSE_RANGE_US = 500        # Half-sweep = 500 µs → ±50°

class SteeringController:
    def __init__(self):
        self.L = WHEELBASE_MM
        self.w = TRACK_WIDTH_MM
        self.servo_angle_limit = SERVO_RANGE_DEG
        self.pulse_center = PULSE_CENTER_US
        self.pulse_range = PULSE_RANGE_US
        self.pulse_min = self.pulse_center - self.pulse_range
        self.pulse_max = self.pulse_center + self.pulse_range

        # Max steering angle from Ackermann geometry (inner wheel turn)
        R_outer = 500 # mm
        R_inner = R_outer - (self.w / 2)
        self.max_steering_angle_deg = math.degrees(math.atan(self.L / R_inner))  # ~27.6°
        
        self.pi = pigpio.pi()
        self.pi.set_mode(GPIO_PIN, pigpio.OUTPUT)

    def angle_to_pwm(self, angle_deg):
        """
        Convert a desired steering angle (±27.6°) into a PWM duty cycle (5% to 10%)
        """
        # Clamp angle to physical limits
        angle_deg = max(-self.max_steering_angle_deg, min(self.max_steering_angle_deg, angle_deg))
        pulse = self.pulse_center + (angle_deg / self.servo_angle_limit) * self.pulse_range
        return int(max(self.pulse_min, min(self.pulse_max, pulse)))

        # Map angle [-max, max] → PWM [min, max]
        #pwm_range = self.servo_pwm_max - self.servo_pwm_min
        #pwm = self.servo_pwm_mid + (angle_deg / (2 * self.max_steering_angle_deg)) * pwm_range * 2
        #return max(self.servo_pwm_min, min(self.servo_pwm_max, pwm))

    def set_steering_angle(self, angle_deg):
        pulse_width = self.angle_to_pwm(angle_deg)
        self.pi.set_servo_pulsewidth(GPIO_PIN, pulse_width)

    def curvature_to_angle(self, curvature):
        """
        Convert a curvature (1/R) into steering angle using Ackermann geometry
        """
        if curvature == 0:
            return 0.0
        radius = 1.0 / curvature
        angle_rad = math.atan(self.L / radius)
        return math.degrees(angle_rad)
    
    def cleanup(self):
        self.pi.set_servo_pulsewidth(GPIO_PIN, 0)
        self.pi.stop()