import RPi.GPIO as GPIO
# for time import sleep
GPIO.setmode(GPIO.BCM)

# Pin defns
# PWM1: speed control left motor
# DIR1: direction control left motor

pwm1 = 14
dir1 = 15
# change these pins on rpi later if need be

# PWM2: speed control right motor
# DIR2: direction control right motor

pwm2 = 23
dir2 = 24

class Motor:
    def __init__(self):
        self.left_dir = dir1 
        self.left_en = pwm1         # speed pin left motor 
        self.right_dir = dir2
        self.right_en = pwm2        # speed pin right motor

        GPIO.setup(self.left_dir,GPIO.OUT)
        GPIO.setup(self.left_en,GPIO.OUT)
        GPIO.setup(self.right_dir,GPIO.OUT)
        GPIO.setup(self.right_en,GPIO.OUT)

        self.left_pwm = GPIO.PWM(self.left_en, 100)     # init pwm for left motor
        self.right_pwm = GPIO.PWM(self.right_en, 100)   # init pwm for right motor

    def forward(self, speed):
        GPIO.output(self.left_dir,GPIO.HIGH)
        GPIO.output(self.right_dir,GPIO.HIGH)
        self.left_pwm.start(speed)
        self.right_pwm.start(speed) 
    # change the wheel directions in code if the polarity wiring is the other way

    def right(self, speed):
        GPIO.output(self.left_dir,GPIO.HIGH)
        GPIO.output(self.right_dir,GPIO.HIGH)
        self.left_pwm.start(speed)
        self.right_pwm.start(speed)

    # left wheel direction = backward
    # right wheel direction = forward
    def left(self, speed):
        GPIO.output(self.left_dir,GPIO.LOW)
        GPIO.output(self.right_dir,GPIO.LOW)
        self.left_pwm.start(speed)
        self.right_pwm.start(speed)

    def back(self, speed):
        GPIO.output(self.left_dir,GPIO.LOW)
        GPIO.output(self.right_dir,GPIO.HIGH)
        self.left_pwm.start(speed)
        self.right_pwm.start(speed)