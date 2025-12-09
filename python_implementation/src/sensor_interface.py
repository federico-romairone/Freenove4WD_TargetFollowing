"""
Sensor Interface Module for Target Detection

This module provides an abstraction layer for sensor input,
particularly for detecting and tracking targets.
"""

import math
import time


class SensorInterface:
    """
    Abstract sensor interface for target detection.
    
    This class should be subclassed to interface with actual sensors
    such as cameras, ultrasonic sensors, or IR sensors.
    """
    
    def __init__(self):
        """Initialize the sensor interface."""
        self.last_detection_time = None
        self.detection_timeout = 5.0  # seconds
        
    def get_target_position(self):
        """
        Get the position of the target relative to the robot.
        
        Returns:
            dict: Dictionary with 'distance', 'angle', and 'detected' keys,
                  or None if no target is detected.
                  - distance: Distance to target in meters
                  - angle: Angle to target in radians (-pi to pi)
                  - detected: Boolean indicating if target is currently visible
        """
        raise NotImplementedError("Subclasses must implement get_target_position()")
    
    def is_target_detected(self):
        """
        Check if a target has been detected recently.
        
        Returns:
            bool: True if target detected within timeout period
        """
        if self.last_detection_time is None:
            return False
        return (time.time() - self.last_detection_time) < self.detection_timeout
    
    def set_detection_timeout(self, timeout):
        """
        Set the timeout for target detection.
        
        Args:
            timeout (float): Timeout in seconds
        """
        self.detection_timeout = timeout


class SimulatedSensor(SensorInterface):
    """
    Simulated sensor for testing without hardware.
    
    This sensor generates synthetic target positions for development
    and testing of the control system.
    """
    
    def __init__(self, target_trajectory_func=None):
        """
        Initialize simulated sensor.
        
        Args:
            target_trajectory_func: Function that returns (x, y) target position
                                   given time as input. If None, uses default.
        """
        super().__init__()
        self.target_trajectory_func = target_trajectory_func
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.start_time = time.time()
        
    def set_robot_pose(self, x, y, theta):
        """
        Update the robot's pose for simulation.
        
        Args:
            x (float): Robot x position in meters
            y (float): Robot y position in meters
            theta (float): Robot orientation in radians
        """
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta
    
    def get_target_position(self):
        """
        Get simulated target position.
        
        Returns:
            dict: Target position relative to robot
        """
        # Get current time
        current_time = time.time() - self.start_time
        
        # Get absolute target position
        if self.target_trajectory_func is not None:
            target_x, target_y = self.target_trajectory_func(current_time)
        else:
            # Default: circular trajectory
            target_x = 2.0 + 1.5 * math.cos(0.3 * current_time)
            target_y = 2.0 + 1.5 * math.sin(0.3 * current_time)
        
        # Calculate relative position
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        
        # Calculate distance
        distance = math.sqrt(dx * dx + dy * dy)
        
        # Calculate angle to target in global frame
        angle_global = math.atan2(dy, dx)
        
        # Convert to robot frame
        angle_relative = angle_global - self.robot_theta
        
        # Normalize angle to [-pi, pi]
        angle_relative = math.atan2(math.sin(angle_relative), math.cos(angle_relative))
        
        self.last_detection_time = time.time()
        
        return {
            'distance': distance,
            'angle': angle_relative,
            'detected': True
        }


class CameraSensor(SensorInterface):
    """
    Camera-based sensor for target detection.
    
    This class provides a template for implementing vision-based
    target detection using a camera (e.g., with OpenCV).
    """
    
    def __init__(self, camera_id=0, target_color=None):
        """
        Initialize camera sensor.
        
        Args:
            camera_id (int): Camera device ID
            target_color (tuple): HSV color range for target detection (optional)
        """
        super().__init__()
        self.camera_id = camera_id
        self.target_color = target_color
        
        # Camera parameters (should be calibrated)
        self.focal_length = 500  # pixels
        self.image_width = 640
        self.image_height = 480
        self.known_target_width = 0.1  # meters (for distance estimation)
        
    def get_target_position(self):
        """
        Detect target using camera and calculate position.
        
        Returns:
            dict: Target position or None if not detected
        """
        # Placeholder for actual camera processing
        # In a real implementation, this would:
        # 1. Capture frame from camera
        # 2. Process image to detect target (color detection, feature matching, etc.)
        # 3. Calculate distance using target size or stereo vision
        # 4. Calculate angle from target position in image
        
        # Example structure:
        # import cv2
        # ret, frame = self.camera.read()
        # if ret:
        #     # Process frame to detect target
        #     target_detected, target_pixel_x, target_pixel_width = self.detect_target(frame)
        #     if target_detected:
        #         # Calculate angle from pixel position
        #         angle = self.calculate_angle_from_pixel(target_pixel_x)
        #         # Calculate distance from target size
        #         distance = self.calculate_distance_from_size(target_pixel_width)
        #         return {'distance': distance, 'angle': angle, 'detected': True}
        
        return None
    
    def calculate_angle_from_pixel(self, pixel_x):
        """
        Calculate angle to target from pixel position.
        
        Args:
            pixel_x (float): Horizontal pixel position of target
            
        Returns:
            float: Angle in radians
        """
        # Angle from camera center
        pixel_offset = pixel_x - (self.image_width / 2)
        angle = math.atan2(pixel_offset, self.focal_length)
        return angle
    
    def calculate_distance_from_size(self, pixel_width):
        """
        Calculate distance to target from its apparent size.
        
        Args:
            pixel_width (float): Width of target in pixels
            
        Returns:
            float: Distance in meters
        """
        if pixel_width > 0:
            distance = (self.known_target_width * self.focal_length) / pixel_width
            return distance
        return float('inf')
