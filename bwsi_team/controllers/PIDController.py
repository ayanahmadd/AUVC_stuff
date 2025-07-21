
class PIDController:
    def __init__(self, kp, ki, kd, setpoint, dt=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.dt = dt

        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, current_depth):
        error = self.setpoint - current_depth
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error

        return output


