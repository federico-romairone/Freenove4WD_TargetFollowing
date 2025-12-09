"""
Configuration Module for Target Following System

This module provides configuration management for the target following system.
"""

import json
import os


DEFAULT_CONFIG = {
    # Control parameters
    'desired_distance': 0.5,  # Desired distance to target (meters)
    'min_distance': 0.2,      # Minimum safe distance (meters)
    'max_distance': 5.0,      # Maximum tracking distance (meters)
    
    # Motor parameters
    'max_forward_speed': 80,  # Maximum forward speed (0-100)
    'max_turn_speed': 60,     # Maximum turning speed (0-100)
    'wheel_base': 0.15,       # Distance between wheels (meters)
    
    # Control loop
    'control_loop_rate': 20,  # Control loop frequency (Hz)
    
    # Distance PID controller
    'distance_pid': {
        'Kp': 50.0,
        'Ki': 1.0,
        'Kd': 10.0
    },
    
    # Angle PID controller
    'angle_pid': {
        'Kp': 80.0,
        'Ki': 1.0,
        'Kd': 15.0
    },
    
    # Sensor parameters
    'sensor_type': 'simulated',  # 'simulated', 'camera', 'ultrasonic'
    'detection_timeout': 5.0,    # Time before giving up on target (seconds)
    
    # Camera parameters (if using camera sensor)
    'camera': {
        'device_id': 0,
        'focal_length': 500,     # pixels
        'image_width': 640,
        'image_height': 480,
        'target_width': 0.1      # Known target width (meters)
    }
}


class ConfigManager:
    """
    Configuration manager for loading and saving settings.
    """
    
    def __init__(self, config_file=None):
        """
        Initialize configuration manager.
        
        Args:
            config_file (str, optional): Path to configuration file
        """
        self.config_file = config_file
        self.config = DEFAULT_CONFIG.copy()
        
        if config_file and os.path.exists(config_file):
            self.load(config_file)
    
    def load(self, config_file):
        """
        Load configuration from JSON file.
        
        Args:
            config_file (str): Path to configuration file
        """
        try:
            with open(config_file, 'r') as f:
                loaded_config = json.load(f)
                self.config.update(loaded_config)
            print(f"Configuration loaded from {config_file}")
        except Exception as e:
            print(f"Error loading configuration: {e}")
            print("Using default configuration")
    
    def save(self, config_file=None):
        """
        Save configuration to JSON file.
        
        Args:
            config_file (str, optional): Path to save configuration.
                                        Uses initial config_file if not specified.
        """
        if config_file is None:
            config_file = self.config_file
        
        if config_file is None:
            raise ValueError("No config file specified")
        
        try:
            with open(config_file, 'w') as f:
                json.dump(self.config, f, indent=4)
            print(f"Configuration saved to {config_file}")
        except Exception as e:
            print(f"Error saving configuration: {e}")
    
    def get(self, key=None):
        """
        Get configuration value or entire configuration.
        
        Args:
            key (str, optional): Configuration key. If None, returns entire config.
            
        Returns:
            Configuration value or entire configuration dict
        """
        if key is None:
            return self.config
        return self.config.get(key)
    
    def set(self, key, value):
        """
        Set configuration value.
        
        Args:
            key (str): Configuration key
            value: Configuration value
        """
        self.config[key] = value
    
    def reset_to_default(self):
        """Reset configuration to default values."""
        self.config = DEFAULT_CONFIG.copy()
        print("Configuration reset to defaults")
