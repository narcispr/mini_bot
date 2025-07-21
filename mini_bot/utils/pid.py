class PID:
    def __init__(self, kp, ki, kd, integral_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < -self.integral_max:
            self.integral = -self.integral_max
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) / dt
        d_term = self.kd * derivative

        # Update previous error
        self.previous_error = error

        # Return PID output
        return p_term + i_term + d_term
