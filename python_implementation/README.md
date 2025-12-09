# Freenove 4WD Target Following - Python Implementation

This directory contains the Python implementation of the closed-loop control system for target following on the Freenove 4WD smart car.

## Overview

The system uses PID (Proportional-Integral-Derivative) controllers to maintain a desired distance and orientation relative to a detected target. The architecture is modular and allows for easy adaptation to different sensor types and hardware configurations.

## Architecture

The system consists of the following main components:

- **PID Controller** (`pid_controller.py`): Implements PID control with anti-windup
- **Motor Controller** (`motor_controller.py`): Abstracts motor control for differential drive
- **Sensor Interface** (`sensor_interface.py`): Provides target detection capabilities
- **Target Follower** (`target_follower.py`): Main control loop integrating all components
- **Configuration Manager** (`config.py`): Handles system configuration

## Installation

1. Install Python 3.7 or higher

2. Install dependencies:
```bash
cd python_implementation
pip install -r requirements.txt
```

3. For hardware implementation, install additional dependencies:
```bash
# For camera-based detection
pip install opencv-python numpy

# For Raspberry Pi GPIO (if applicable)
pip install RPi.GPIO adafruit-circuitpython-pca9685
```

## Usage

### Running the Simulation

The basic simulation uses simulated sensors and motors for testing:

```bash
cd examples
python3 basic_simulation.py
```

This will run a 20-second simulation with a moving target and display real-time status updates.

### Hardware Implementation

1. Edit `examples/hardware_example.py` to implement your specific hardware interfaces:
   - Customize `Freenove4WDMotorController` for your motor driver
   - Customize `Freenove4WDTargetSensor` for your camera/sensor setup

2. Create a configuration file (`config.json`):
```json
{
    "desired_distance": 0.5,
    "max_forward_speed": 80,
    "max_turn_speed": 60,
    "control_loop_rate": 20,
    "distance_pid": {
        "Kp": 50.0,
        "Ki": 1.0,
        "Kd": 10.0
    },
    "angle_pid": {
        "Kp": 80.0,
        "Ki": 1.0,
        "Kd": 15.0
    }
}
```

3. Run the hardware example:
```bash
python3 examples/hardware_example.py
```

## Configuration

### PID Parameters

The system uses two PID controllers:

1. **Distance Controller**: Controls forward/backward speed to maintain desired distance
   - `Kp_dist`: Proportional gain (default: 50.0)
   - `Ki_dist`: Integral gain (default: 1.0)
   - `Kd_dist`: Derivative gain (default: 10.0)

2. **Angle Controller**: Controls turning to face the target
   - `Kp_angle`: Proportional gain (default: 80.0)
   - `Ki_angle`: Integral gain (default: 1.0)
   - `Kd_angle`: Derivative gain (default: 15.0)

### Tuning Guidelines

- **If the robot oscillates**: Reduce `Kp`, increase `Kd`
- **If the robot is too slow to respond**: Increase `Kp`
- **If there is steady-state error**: Increase `Ki` (carefully, to avoid windup)
- **For aggressive tracking**: Increase all gains proportionally
- **For smooth, conservative tracking**: Decrease all gains proportionally

## Testing

Run the unit tests:

```bash
cd python_implementation
python -m pytest tests/ -v
```

Run specific test files:

```bash
python -m pytest tests/test_pid_controller.py -v
python -m pytest tests/test_motor_controller.py -v
```

## Hardware Integration Guide

### Motor Control

The `MotorController` class needs to be extended to interface with your specific hardware:

```python
class MyMotorController(MotorController):
    def _apply_motor_commands(self, left_speed, right_speed):
        # Add your hardware-specific code here
        # Example for Freenove library:
        # self.motor_driver.set_speeds(left_speed, right_speed)
        pass
```

### Sensor Integration

For camera-based target detection:

```python
class MyTargetSensor(CameraSensor):
    def get_target_position(self):
        # Capture frame
        # Detect target (color, shape, or ML-based)
        # Calculate distance and angle
        # Return {'distance': d, 'angle': a, 'detected': True}
        pass
```

## Project Structure

```
python_implementation/
├── src/
│   ├── __init__.py
│   ├── pid_controller.py       # PID controller implementation
│   ├── motor_controller.py     # Motor control abstraction
│   ├── sensor_interface.py     # Sensor abstraction
│   ├── target_follower.py      # Main control system
│   └── config.py               # Configuration management
├── tests/
│   ├── __init__.py
│   ├── test_pid_controller.py
│   └── test_motor_controller.py
├── examples/
│   ├── basic_simulation.py     # Simulation example
│   └── hardware_example.py     # Hardware template
├── requirements.txt
└── README.md
```

## Troubleshooting

### Robot doesn't move
- Check motor controller initialization
- Verify motor speed limits are not too low
- Ensure target is detected (check sensor output)

### Robot oscillates around target
- Reduce PID gains (especially Kp)
- Increase damping (Kd)
- Check control loop rate (may be too slow)

### Robot doesn't track target accurately
- Increase PID gains
- Check sensor accuracy and update rate
- Verify wheel base parameter matches hardware

### Target detection issues
- Check camera/sensor initialization
- Verify target is visible and distinguishable
- Adjust detection parameters (color range, size thresholds)

## License

This project is provided as-is for educational and research purposes.

## Contributing

Feel free to submit issues, feature requests, or pull requests to improve the system.
