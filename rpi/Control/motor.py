import RPi.GPIO as GPIO

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
    def __init__(self, base_speed, min_speed):
        """
        Initialize motor controller
        Args:
            base_speed (int): Default speed (0-100)
            min_speed (int): Minimum speed when turning (0-100)
        """
        # Store speed parameters
        self.base_speed = base_speed
        self.min_speed = min_speed

        GPIO.setwarnings(False)  # Suppress "already in use" warnings
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([PWM_LEFT, DIR_LEFT, PWM_RIGHT, DIR_RIGHT], GPIO.OUT)

    
        self.left_pwm = GPIO.PWM(PWM_LEFT, 100)     # init pwm for left motor

    
        self.right_pwm = GPIO.PWM(PWM_RIGHT, 100)   # init pwm for right motor

        self.left_pwm.start(0)
        self.right_pwm.start(0)

        # Set initial direction forward
        GPIO.output(DIR_LEFT, GPIO.HIGH)
        GPIO.output(DIR_RIGHT, GPIO.HIGH)

    def move_scaled(self, steering_angle, max_steering_angle):
        """
        Adjust motor speed based on steering angle
        Args:
            steering_angle (float): Current steering angle
            max_steering_angle (float): Vehicle's max steering angle
        """
        # Reduce speed proportionally to steering angle
        factor = 1 - (abs(steering_angle) / max_steering_angle)
        speed = max(self.min_speed, self.base_speed * factor)
        self.forward(speed)

    def forward(self, speed):
        """
        Move forward at specified speed
        
        Args:
            speed (int): 0-100 percentage of max speed
        """
        speed = max(0, min(100, speed))  # clamp to valid range
        
        # set direction forward
        GPIO.output(DIR_LEFT,GPIO.LOW)
        GPIO.output(DIR_RIGHT,GPIO.LOW)

        # set PWM duty cycle
        self.left_pwm.ChangeDutyCycle(speed)
        self.right_pwm.ChangeDutyCycle(speed)

    # change the wheel directions in code if the polarity wiring is the other way

    def reverse(self, speed):
        """
        Move backward at specified speed
        
        Args:
            speed (int): 0-100 percentage of max speed
        """
        speed = max(0, min(100, speed))
        
        # Set direction reverse
        GPIO.output(DIR_LEFT, GPIO.HIGH)
        GPIO.output(DIR_RIGHT, GPIO.HIGH)
        
        self.left_pwm.ChangeDutyCycle(speed)
        self.right_pwm.ChangeDutyCycle(speed)

    def right(self, speed, reverse_flag=False):
        speed = max(0, min(100, speed))
        if reverse_flag:
            # Right forward, left reverse
            GPIO.output(DIR_LEFT, GPIO.HIGH)
            GPIO.output(DIR_RIGHT, GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(speed)
            self.right_pwm.ChangeDutyCycle(speed)
        else:
            # print("INSIDE DUF RIGHT")
            # Right forward, left slower forward
            GPIO.output(DIR_LEFT, GPIO.LOW)
            GPIO.output(DIR_RIGHT, GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(speed)
            self.right_pwm.ChangeDutyCycle(0)

    
    def left(self, speed, reverse_flag=False):
        speed = max(0, min(100, speed))
        if reverse_flag:
            # Right forward, left reverse
            GPIO.output(DIR_LEFT, GPIO.LOW)
            GPIO.output(DIR_RIGHT, GPIO.HIGH)
            self.left_pwm.ChangeDutyCycle(speed)
            self.right_pwm.ChangeDutyCycle(speed)
        else:
            # print("INSIDE DIF LEFT")
            # Right forward, left slower forward
            GPIO.output(DIR_LEFT, GPIO.LOW)
            GPIO.output(DIR_RIGHT, GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(0)
            self.right_pwm.ChangeDutyCycle(speed)


        
    

        

    def stop(self):
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)

    def cleanup(self):
        self.stop()
        GPIO.cleanup()