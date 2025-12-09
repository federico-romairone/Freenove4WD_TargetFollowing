"""
Target Following System Main Controller

This module implements the main control loop for the target following system,
integrating PID controllers, sensor input, and motor control.
"""

import math
import time
from pid_controller import PIDController
from motor_controller import MotorController
from sensor_interface import SensorInterface


class TargetFollower:
    """
    Main controller for target following behavior.
    
    This class integrates sensor input, PID control, and motor commands
    to make the robot follow a detected target.
    """
    
    def __init__(self, sensor, motor_controller, config=None):
        """
        Initialize the target follower.
        
        Args:
            sensor (SensorInterface): Sensor for target detection
            motor_controller (MotorController): Motor controller for robot motion
            config (dict, optional): Configuration parameters
        """
        self.sensor = sensor
        self.motor_controller = motor_controller
        
        # Load configuration
        if config is None:
            config = self._default_config()
        self.config = config
        
        # Create PID controllers
        self.distance_controller = PIDController(
            Kp=config['distance_pid']['Kp'],
            Ki=config['distance_pid']['Ki'],
            Kd=config['distance_pid']['Kd'],
            output_limit=config['max_forward_speed']
        )
        
        self.angle_controller = PIDController(
            Kp=config['angle_pid']['Kp'],
            Ki=config['angle_pid']['Ki'],
            Kd=config['angle_pid']['Kd'],
            output_limit=config['max_turn_speed']
        )
        
        # State variables
        self.running = False
        self.desired_distance = config['desired_distance']
        self.control_loop_rate = config['control_loop_rate']
        
    @staticmethod
    def _default_config():
        """
        Get default configuration parameters.
        
        Returns:
            dict: Default configuration
        """
        return {
            'desired_distance': 0.5,  # meters
            'max_forward_speed': 80,  # motor speed units (0-100)
            'max_turn_speed': 60,
            'control_loop_rate': 20,  # Hz
            'distance_pid': {
                'Kp': 50.0,
                'Ki': 1.0,
                'Kd': 10.0
            },
            'angle_pid': {
                'Kp': 80.0,
                'Ki': 1.0,
                'Kd': 15.0
            },
            'min_distance': 0.2,  # Stop if closer than this
            'max_distance': 5.0   # Stop tracking if farther than this
        }
    
    def start(self):
        """Start the target following control loop."""
        self.running = True
        self.distance_controller.reset()
        self.angle_controller.reset()
        
        print("Target following started")
        print(f"Desired distance: {self.desired_distance} m")
        print(f"Control loop rate: {self.control_loop_rate} Hz")
        
        dt = 1.0 / self.control_loop_rate
        
        try:
            while self.running:
                loop_start = time.time()
                
                # Get target position from sensor
                target_info = self.sensor.get_target_position()
                
                if target_info is None or not target_info['detected']:
                    # No target detected - stop and search
                    self._handle_no_target()
                else:
                    # Target detected - compute control
                    self._control_step(target_info, dt)
                
                # Maintain control loop rate
                loop_duration = time.time() - loop_start
                sleep_time = dt - loop_duration
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            print("\nTarget following stopped by user")
        finally:
            self.stop()
    
    def _control_step(self, target_info, dt):
        """
        Execute one step of the control loop.
        
        Args:
            target_info (dict): Target position information
            dt (float): Time step in seconds
        """
        distance = target_info['distance']
        angle = target_info['angle']
        
        # Check distance bounds
        if distance < self.config['min_distance']:
            # Too close - back up
            self.motor_controller.move_backward(30)
            return
        elif distance > self.config['max_distance']:
            # Too far - stop tracking
            self._handle_no_target()
            return
        
        # Calculate errors
        distance_error = distance - self.desired_distance
        angle_error = angle
        
        # Compute control outputs
        forward_speed = self.distance_controller.compute(distance_error, dt)
        turn_speed = self.angle_controller.compute(angle_error, dt)
        
        # Convert to wheel speeds (differential drive)
        left_speed = forward_speed - turn_speed
        right_speed = forward_speed + turn_speed
        
        # Apply motor commands
        self.motor_controller.set_motor_speeds(left_speed, right_speed)
        
        # Debug output (optional)
        if hasattr(self, 'debug') and self.debug:
            print(f"Distance: {distance:.2f}m, Angle: {math.degrees(angle):.1f}°, "
                  f"Speeds: L={left_speed:.1f}, R={right_speed:.1f}")
    
    def _handle_no_target(self):
        """Handle the case when no target is detected."""
        # Stop motors or implement search behavior
        self.motor_controller.stop()
    
    def stop(self):
        """Stop the target following system."""
        self.running = False
        self.motor_controller.stop()
        print("Target following stopped")
    
    def set_desired_distance(self, distance):
        """
        Set the desired following distance.
        
        Args:
            distance (float): Desired distance in meters
        """
        self.desired_distance = distance
        print(f"Desired distance set to {distance} m")
    
    def set_pid_gains(self, controller_type, Kp=None, Ki=None, Kd=None):
        """
        Update PID gains for a controller.
        
        Args:
            controller_type (str): 'distance' or 'angle'
            Kp (float, optional): Proportional gain
            Ki (float, optional): Integral gain
            Kd (float, optional): Derivative gain
        """
        if controller_type == 'distance':
            self.distance_controller.set_gains(Kp, Ki, Kd)
        elif controller_type == 'angle':
            self.angle_controller.set_gains(Kp, Ki, Kd)
        else:
            raise ValueError("controller_type must be 'distance' or 'angle'")
        
        print(f"Updated {controller_type} PID gains")
    
    def enable_debug(self, enable=True):
        """
        Enable or disable debug output.
        
        Args:
            enable (bool): True to enable debug output
        """
        self.debug = enable
