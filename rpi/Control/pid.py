class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Controller state
        self.prev_error = 0
        self.integral = 0
      
    def compute(self, error):
        prop = self.Kp * error 
        
        self.integral += error
        integral = self.Ki * self.integral 

        derivative = (error - self.prev_error) * self.Kd

        output = prop + integral + derivative

        self.prev_error = error
        return output
