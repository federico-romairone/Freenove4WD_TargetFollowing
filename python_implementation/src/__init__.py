"""
Freenove 4WD Target Following System

A closed-loop control system for target following using PID controllers.
"""

from .pid_controller import PIDController
from .motor_controller import MotorController, SimulatedMotorController
from .sensor_interface import SensorInterface, SimulatedSensor, CameraSensor
from .target_follower import TargetFollower
from .config import ConfigManager, DEFAULT_CONFIG

__version__ = '1.0.0'
__all__ = [
    'PIDController',
    'MotorController',
    'SimulatedMotorController',
    'SensorInterface',
    'SimulatedSensor',
    'CameraSensor',
    'TargetFollower',
    'ConfigManager',
    'DEFAULT_CONFIG'
]
