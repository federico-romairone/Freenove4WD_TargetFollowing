#!/usr/bin/env python3
"""
Basic Simulation Example

This script demonstrates the target following system using simulated
sensors and motors. Useful for testing and development.
"""

import sys
import os
import time
import math

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from pid_controller import PIDController
from motor_controller import SimulatedMotorController
from sensor_interface import SimulatedSensor
from target_follower import TargetFollower


def simulate_robot_motion(robot_state, motor_speeds, dt, wheel_base=0.15):
    """
    Simulate robot motion based on motor speeds.
    
    Args:
        robot_state (dict): Current robot state with 'x', 'y', 'theta'
        motor_speeds (tuple): (left_speed, right_speed) normalized to 0-1
        dt (float): Time step
        wheel_base (float): Distance between wheels
        
    Returns:
        dict: Updated robot state
    """
    left_speed, right_speed = motor_speeds
    
    # Convert motor speeds (0-100) to velocity (m/s)
    max_velocity = 0.3  # m/s
    v_left = (left_speed / 100.0) * max_velocity
    v_right = (right_speed / 100.0) * max_velocity
    
    # Calculate linear and angular velocities
    v = (v_left + v_right) / 2.0
    omega = (v_right - v_left) / wheel_base
    
    # Update position
    x = robot_state['x'] + v * math.cos(robot_state['theta']) * dt
    y = robot_state['y'] + v * math.sin(robot_state['theta']) * dt
    theta = robot_state['theta'] + omega * dt
    
    # Normalize angle
    theta = math.atan2(math.sin(theta), math.cos(theta))
    
    return {'x': x, 'y': y, 'theta': theta}


def main():
    """Run basic simulation."""
    print("=" * 60)
    print("Freenove 4WD Target Following - Basic Simulation")
    print("=" * 60)
    print()
    
    # Create simulated components
    sensor = SimulatedSensor()
    motor_controller = SimulatedMotorController(max_speed=100, wheel_base=0.15)
    
    # Create target follower with custom configuration
    config = {
        'desired_distance': 0.5,
        'max_forward_speed': 80,
        'max_turn_speed': 60,
        'control_loop_rate': 20,
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
        'min_distance': 0.2,
        'max_distance': 5.0
    }
    
    follower = TargetFollower(sensor, motor_controller, config)
    follower.enable_debug(True)
    
    # Simulation state
    robot_state = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
    
    # Simulation parameters
    dt = 1.0 / config['control_loop_rate']
    simulation_time = 20.0  # seconds
    steps = int(simulation_time / dt)
    
    print(f"Running simulation for {simulation_time} seconds...")
    print(f"Time step: {dt:.3f} s")
    print(f"Total steps: {steps}")
    print()
    print("Press Ctrl+C to stop early")
    print()
    
    try:
        for step in range(steps):
            # Update sensor with current robot pose
            sensor.set_robot_pose(robot_state['x'], robot_state['y'], robot_state['theta'])
            
            # Get target position
            target_info = sensor.get_target_position()
            
            if target_info and target_info['detected']:
                # Calculate errors
                distance_error = target_info['distance'] - config['desired_distance']
                angle_error = target_info['angle']
                
                # Compute control (using follower's controllers)
                forward_speed = follower.distance_controller.compute(distance_error, dt)
                turn_speed = follower.angle_controller.compute(angle_error, dt)
                
                # Convert to wheel speeds
                left_speed = forward_speed - turn_speed
                right_speed = forward_speed + turn_speed
                
                # Apply to motor controller
                motor_controller.set_motor_speeds(left_speed, right_speed)
                
                # Simulate robot motion
                motor_speeds = motor_controller.get_current_speeds()
                robot_state = simulate_robot_motion(robot_state, motor_speeds, dt)
                
                # Print status every second
                if step % config['control_loop_rate'] == 0:
                    t = step * dt
                    print(f"t={t:.1f}s: Pos=({robot_state['x']:.2f}, {robot_state['y']:.2f}), "
                          f"Dist={target_info['distance']:.2f}m, "
                          f"Angle={math.degrees(target_info['angle']):.1f}°")
            else:
                motor_controller.stop()
            
            # Small delay to prevent CPU overuse
            time.sleep(dt / 10)  # Run faster than real-time for simulation
            
    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user")
    
    # Final statistics
    print()
    print("=" * 60)
    print("Simulation Complete")
    print("=" * 60)
    print(f"Final robot position: ({robot_state['x']:.2f}, {robot_state['y']:.2f})")
    print(f"Final robot orientation: {math.degrees(robot_state['theta']):.1f}°")
    
    history = motor_controller.get_command_history()
    print(f"Total motor commands issued: {len(history)}")


if __name__ == '__main__':
    main()
