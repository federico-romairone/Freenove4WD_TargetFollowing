"""
Motor Controller Module for Freenove 4WD Smart Car

This module provides an abstraction layer for controlling the motors
of the Freenove 4WD smart car.
"""

import time


class MotorController:
    """
    Motor controller for differential drive robot.
    
    This class provides methods to control the motors of a differential
    drive robot. It can be subclassed to interface with actual hardware.
    """
    
    def __init__(self, max_speed=100, wheel_base=0.15):
        """
        Initialize the motor controller.
        
        Args:
            max_speed (float): Maximum motor speed (0-100 for PWM, or m/s for velocity)
            wheel_base (float): Distance between left and right wheels (meters)
        """
        self.max_speed = max_speed
        self.wheel_base = wheel_base
        self.current_left_speed = 0
        self.current_right_speed = 0
        
    def set_motor_speeds(self, left_speed, right_speed):
        """
        Set the speed of left and right motors.
        
        Args:
            left_speed (float): Speed for left motors (-max_speed to +max_speed)
            right_speed (float): Speed for right motors (-max_speed to +max_speed)
        """
        # Clamp speeds to valid range
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        
        self.current_left_speed = left_speed
        self.current_right_speed = right_speed
        
        # In a real implementation, this would send commands to the motor driver
        # For example, using GPIO or I2C communication
        self._apply_motor_commands(left_speed, right_speed)
    
    def _apply_motor_commands(self, left_speed, right_speed):
        """
        Apply motor commands to hardware (to be overridden in subclass).
        
        Args:
            left_speed (float): Speed for left motors
            right_speed (float): Speed for right motors
        """
        # Placeholder for actual hardware control
        # In a real implementation, this would interface with:
        # - GPIO pins for PWM control
        # - Motor driver IC (e.g., L298N, PCA9685)
        # - Freenove's specific motor control library
        pass
    
    def stop(self):
        """Stop all motors."""
        self.set_motor_speeds(0, 0)
    
    def move_forward(self, speed):
        """
        Move forward at specified speed.
        
        Args:
            speed (float): Forward speed (0 to max_speed)
        """
        self.set_motor_speeds(speed, speed)
    
    def move_backward(self, speed):
        """
        Move backward at specified speed.
        
        Args:
            speed (float): Backward speed (0 to max_speed)
        """
        self.set_motor_speeds(-speed, -speed)
    
    def turn_left(self, speed):
        """
        Turn left (rotate counter-clockwise).
        
        Args:
            speed (float): Turning speed (0 to max_speed)
        """
        self.set_motor_speeds(-speed, speed)
    
    def turn_right(self, speed):
        """
        Turn right (rotate clockwise).
        
        Args:
            speed (float): Turning speed (0 to max_speed)
        """
        self.set_motor_speeds(speed, -speed)
    
    def get_current_speeds(self):
        """
        Get current motor speeds.
        
        Returns:
            tuple: (left_speed, right_speed)
        """
        return (self.current_left_speed, self.current_right_speed)


class SimulatedMotorController(MotorController):
    """
    Simulated motor controller for testing without hardware.
    
    This class extends MotorController to simulate motor behavior
    for development and testing purposes.
    """
    
    def __init__(self, max_speed=100, wheel_base=0.15):
        """Initialize simulated motor controller."""
        super().__init__(max_speed, wheel_base)
        self.command_history = []
    
    def _apply_motor_commands(self, left_speed, right_speed):
        """
        Simulate motor command application.
        
        Args:
            left_speed (float): Speed for left motors
            right_speed (float): Speed for right motors
        """
        timestamp = time.time()
        self.command_history.append({
            'time': timestamp,
            'left_speed': left_speed,
            'right_speed': right_speed
        })
        
        # Keep only recent history (last 1000 commands)
        if len(self.command_history) > 1000:
            self.command_history.pop(0)
    
    def get_command_history(self):
        """
        Get motor command history.
        
        Returns:
            list: History of motor commands with timestamps
        """
        return self.command_history
