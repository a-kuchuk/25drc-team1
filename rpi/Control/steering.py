import math
import pigpio

# Constants
GPIO_PIN = 26               # GPIO pin connected to servo
WHEELBASE_MM = 220          # Distance between front and rear axles (mm)
TRACK_WIDTH_MM = 151        # Distance between left and right wheels (mm)

# Servo Parameters
SERVO_SPEED = 0.09          # 0.09sec/60° @ 7.4V
SERVO_TORQUE = 5.5          # 5.5Kgf.cm @ 7.4V 

# Servo Parameters
SERVO_MAX_ANGLE = 45              # ±45° safe mechanical range
PULSE_CENTER_US = 1520            # Center pulse width (µs)
PULSE_RANGE_US = 450              # Half sweep = ±450 µs for ±45°
PULSE_MIN_US = PULSE_CENTER_US - PULSE_RANGE_US  # 1070 µs
PULSE_MAX_US = PULSE_CENTER_US + PULSE_RANGE_US  # 1970 µs

class SteeringController:
    def __init__(self):
        # self.L = WHEELBASE_MM
        # self.w = TRACK_WIDTH_MM
        # # Max steering angle from Ackermann geometry (inner wheel turn)
        # R_outer = 500 # mm
        # R_inner = R_outer - (self.w / 2)
        # self.max_steering_angle_deg = math.degrees(math.atan(self.L / R_inner))  # ~27.6°

        self.max_steering_angle_deg = SERVO_MAX_ANGLE
        self.pulse_center = PULSE_CENTER_US
        self.pulse_range = PULSE_RANGE_US
        self.pulse_min = PULSE_MIN_US
        self.pulse_max = PULSE_MAX_US

        # Initialize pigpio for hardware PWM
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon")
        self.pi.set_mode(GPIO_PIN, pigpio.OUTPUT)

    def angle_to_pwm(self, angle_deg):
        """
        Convert a steering angle to PWM pulse width (µs), clamped to safe limits.
        """
        # Clamp angle to physical limits
        angle_deg = max(-self.max_steering_angle_deg, 
                        min(self.max_steering_angle_deg, angle_deg))
               
        pulse = self.pulse_center + (angle_deg / self.max_steering_angle_deg) * self.pulse_range
        # ensure pulse stays with servo's acceptable range
        return int(max(self.pulse_min,xmin(self.pulse_max, pulse)))

    def set_steering_angle(self, angle_deg):
        """
        Set the steering angle using Ackermann-corrected servo position
        Args:
            angle_deg (float): Desired steering angle in degrees
        """
        pulse_width = self.angle_to_pwm(angle_deg)
        self.pi.set_servo_pulsewidth(GPIO_PIN, pulse_width)
    
    def cleanup(self):
        self.pi.set_servo_pulsewidth(GPIO_PIN, 0)
        self.pi.stop()