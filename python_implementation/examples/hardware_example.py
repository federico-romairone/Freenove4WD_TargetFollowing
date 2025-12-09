#!/usr/bin/env python3
"""
Hardware Implementation Example

This script demonstrates how to use the target following system with
actual Freenove 4WD hardware. You need to implement the hardware-specific
interfaces for your sensors and motors.
"""

import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from motor_controller import MotorController
from sensor_interface import CameraSensor
from target_follower import TargetFollower
from config import ConfigManager


class Freenove4WDMotorController(MotorController):
    """
    Motor controller implementation for Freenove 4WD hardware.
    
    This class should be customized based on your specific hardware setup.
    You may need to install and use the Freenove Python library.
    """
    
    def __init__(self, max_speed=100, wheel_base=0.15):
        """Initialize Freenove 4WD motor controller."""
        super().__init__(max_speed, wheel_base)
        
        # Initialize hardware interface
        # Example (adjust based on actual Freenove library):
        # from Freenove_Robot_Car_for_Raspberry_Pi.Code.Server import Motor
        # self.motor = Motor.Motor()
        
        print("Initializing Freenove 4WD motor controller...")
        # Add your hardware initialization code here
    
    def _apply_motor_commands(self, left_speed, right_speed):
        """
        Apply motor commands to Freenove 4WD hardware.
        
        Args:
            left_speed (float): Speed for left motors (-100 to 100)
            right_speed (float): Speed for right motors (-100 to 100)
        """
        # Convert speeds to appropriate format for your hardware
        # Example (adjust based on actual Freenove library):
        # if left_speed >= 0:
        #     self.motor.setMotorModel(left_speed, left_speed, right_speed, right_speed)
        # else:
        #     self.motor.setMotorModel(-left_speed, -left_speed, -right_speed, -right_speed)
        
        pass  # Replace with actual hardware control code


class Freenove4WDTargetSensor(CameraSensor):
    """
    Camera sensor implementation for Freenove 4WD with target detection.
    
    This class implements actual computer vision for target detection.
    """
    
    def __init__(self, camera_id=0):
        """Initialize camera sensor."""
        super().__init__(camera_id)
        
        # Initialize camera
        # Example using OpenCV:
        # import cv2
        # self.camera = cv2.VideoCapture(camera_id)
        
        print("Initializing camera sensor...")
        # Add your camera initialization code here
    
    def get_target_position(self):
        """
        Detect target using camera and calculate position.
        
        Returns:
            dict: Target position or None if not detected
        """
        # Implement actual target detection
        # Example approach:
        # 1. Capture frame
        # 2. Apply color filtering or object detection
        # 3. Calculate target centroid and size
        # 4. Estimate distance and angle
        
        # Example pseudocode:
        # import cv2
        # ret, frame = self.camera.read()
        # if not ret:
        #     return None
        #
        # # Convert to HSV for color detection
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #
        # # Define color range for target (e.g., red object)
        # lower_red = np.array([0, 100, 100])
        # upper_red = np.array([10, 255, 255])
        # mask = cv2.inRange(hsv, lower_red, upper_red)
        #
        # # Find contours
        # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #
        # if contours:
        #     # Get largest contour
        #     largest_contour = max(contours, key=cv2.contourArea)
        #     M = cv2.moments(largest_contour)
        #
        #     if M["m00"] > 0:
        #         # Calculate centroid
        #         cx = int(M["m10"] / M["m00"])
        #         
        #         # Calculate bounding box width
        #         x, y, w, h = cv2.boundingRect(largest_contour)
        #         
        #         # Calculate angle and distance
        #         angle = self.calculate_angle_from_pixel(cx)
        #         distance = self.calculate_distance_from_size(w)
        #         
        #         self.last_detection_time = time.time()
        #         return {
        #             'distance': distance,
        #             'angle': angle,
        #             'detected': True
        #         }
        
        return None


def main():
    """Main function for hardware implementation."""
    print("=" * 60)
    print("Freenove 4WD Target Following - Hardware Implementation")
    print("=" * 60)
    print()
    
    # Load configuration
    config_manager = ConfigManager('config.json')
    config = config_manager.get()
    
    print("Configuration loaded:")
    print(f"  Desired distance: {config['desired_distance']} m")
    print(f"  Control loop rate: {config['control_loop_rate']} Hz")
    print()
    
    # Initialize hardware components
    print("Initializing hardware...")
    
    # Create motor controller
    motor_controller = Freenove4WDMotorController(
        max_speed=config['max_forward_speed'],
        wheel_base=config['wheel_base']
    )
    
    # Create sensor
    sensor = Freenove4WDTargetSensor(
        camera_id=config['camera']['device_id']
    )
    
    # Create target follower
    follower = TargetFollower(sensor, motor_controller, config)
    follower.enable_debug(True)
    
    print()
    print("Hardware initialized successfully!")
    print()
    print("Starting target following system...")
    print("The robot will now follow detected targets.")
    print("Press Ctrl+C to stop.")
    print()
    
    try:
        # Start the target following loop
        follower.start()
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        follower.stop()
        print("System stopped.")


if __name__ == '__main__':
    main()
