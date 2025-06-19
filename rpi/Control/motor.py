import RPi.GPIO as GPIO
# for time import sleep
GPIO.setmode(GPIO.BCM)

# Pin defns
# PWM1: speed control left motor
# DIR1: direction control left motor
# change these pins on rpi later if need be
# PWM2: speed control right motor
# DIR2: direction control right motor

PWM_LEFT = 14
DIR_LEFT = 15
PWM_RIGHT = 23
DIR_RIGHT = 24

class Motor:
    def __init__(self):
        self.left_dir = DIR_LEFT
        self.left_en = PWM_LEFT
        self.right_dir = DIR_RIGHT
        self.right_en = PWM_RIGHT   

        GPIO.setup(self.left_dir,GPIO.OUT)
        GPIO.setup(self.left_en,GPIO.OUT)
        GPIO.setup(self.right_dir,GPIO.OUT)
        GPIO.setup(self.right_en,GPIO.OUT)

        self.left_pwm = GPIO.PWM(self.left_en, 100)     # init pwm for left motor
        self.right_pwm = GPIO.PWM(self.right_en, 100)   # init pwm for right motor

        self.left_pwm.start(0)
        self.right_pwm.start(0)

    def move_scaled(self, steering_angle, max_steering_angle):
        """
        Adjust motor speed based on steering angle
        """
        factor = 1 - (abs(steering_angle) / max_steering_angle)
        speed = self.base_speed * factor
        speed = max(self.min_speed, min(self.base_speed, speed))

        GPIO.output(DIR_LEFT, GPIO.HIGH)
        GPIO.output(DIR_RIGHT, GPIO.HIGH)
        self.left_pwm.ChangeDutyCycle(speed)
        self.right_pwm.ChangeDutyCycle(speed)

    def forward(self, speed):
        speed = max(0, min(100, speed))  # clamp
        GPIO.output(self.left_dir,GPIO.HIGH)
        GPIO.output(self.right_dir,GPIO.HIGH)
        self.left_pwm.ChangeDutyCycle(speed)
        self.right_pwm.ChangeDutyCycle(speed)

    # change the wheel directions in code if the polarity wiring is the other way

    def back(self, speed):
        GPIO.output(self.left_dir,GPIO.LOW)
        GPIO.output(self.right_dir,GPIO.HIGH)
        self.left_pwm.start(speed)
        self.right_pwm.start(speed)

    def stop(self):
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)