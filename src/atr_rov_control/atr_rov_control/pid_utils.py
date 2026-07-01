import math 

class PID:
    """
    PID controller with:
    - Anti-windup integral clamping
    - Derivative low-pass filter (avoids derivative kick on setpoint changes)
    - Output saturation
    """
    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 output_min=-float('inf'), output_max=float('inf'),
                 integral_min=-float('inf'), integral_max=float('inf'),
                 derivative_filter_coeff=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.alpha = derivative_filter_coeff # 0=1: no filtering, 1: full filtering

        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_derivative = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_derivative = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0
        
        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup clamping
        self._integral += error * dt
        self._integral = max(self.integral_min, min(self.integral_max, self._integral))
        i_term = self.ki * self._integral

        # Derivative term with low-pass filter (on error, not setpoint)
        raw_derivative = (error - self._prev_error) / dt
        self._filtered_derivative = (self.alpha * raw_derivative + (1.0 - self.alpha) * self._filtered_derivative)
        d_term = self.kd * self._filtered_derivative

        # Update previous error
        self._prev_error = error

        # Calculate total output and apply saturation limits
        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))

        return output
    
    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd