"""
PID Controller Implementation for Target Following System

This module provides a PID (Proportional-Integral-Derivative) controller
for closed-loop control systems.
"""

import time


class PIDController:
    """
    A PID controller implementation with anti-windup.
    
    Attributes:
        Kp (float): Proportional gain
        Ki (float): Integral gain
        Kd (float): Derivative gain
        output_limit (float): Maximum absolute output value
    """
    
    def __init__(self, Kp, Ki, Kd, output_limit=1.0):
        """
        Initialize the PID controller.
        
        Args:
            Kp (float): Proportional gain
            Ki (float): Integral gain
            Kd (float): Derivative gain
            output_limit (float): Maximum absolute output value
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limit = output_limit
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
    def compute(self, error, dt=None):
        """
        Compute the PID control output.
        
        Args:
            error (float): Current error (setpoint - measured value)
            dt (float, optional): Time step in seconds. If None, uses system time.
            
        Returns:
            float: Control output limited by output_limit
        """
        # Calculate time step if not provided
        if dt is None:
            current_time = time.time()
            if self.prev_time is None:
                dt = 0.0
                self.prev_time = current_time
            else:
                dt = current_time - self.prev_time
                self.prev_time = current_time
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with anti-windup
        if dt > 0:
            self.integral += error * dt
            # Anti-windup: limit integral term
            if self.Ki != 0:
                max_integral = self.output_limit / self.Ki
                self.integral = max(min(self.integral, max_integral), -max_integral)
        I = self.Ki * self.integral
        
        # Derivative term
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        D = self.Kd * derivative
        
        # Calculate total output
        output = P + I + D
        
        # Apply output limits
        output = max(min(output, self.output_limit), -self.output_limit)
        
        # Store error for next iteration
        self.prev_error = error
        
        return output
    
    def reset(self):
        """Reset the controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
    
    def set_gains(self, Kp=None, Ki=None, Kd=None):
        """
        Update PID gains.
        
        Args:
            Kp (float, optional): New proportional gain
            Ki (float, optional): New integral gain
            Kd (float, optional): New derivative gain
        """
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd
    
    def get_gains(self):
        """
        Get current PID gains.
        
        Returns:
            tuple: (Kp, Ki, Kd)
        """
        return (self.Kp, self.Ki, self.Kd)
