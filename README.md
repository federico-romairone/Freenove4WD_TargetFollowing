# Freenove 4WD Target Following System

A comprehensive closed-loop control system for autonomous target following using the Freenove 4WD smart car. This project includes both MATLAB simulation for controller design and Python implementation for hardware deployment.

## Overview

This project implements a PID-based control system that enables a Freenove 4WD robot to autonomously follow a detected target while maintaining a desired distance and orientation. The system uses:

- **Dual PID Controllers**: Separate controllers for distance and angle control
- **Differential Drive Kinematics**: Accurate vehicle model for 4-wheel differential drive
- **Modular Architecture**: Easy to adapt to different sensors and hardware configurations
- **Simulation-First Approach**: Test and tune parameters in MATLAB before hardware deployment

## Features

✅ **MATLAB Simulation**
- Complete kinematic vehicle model
- PID controller implementation with anti-windup
- Real-time visualization of trajectories and errors
- Interactive parameter tuning tools
- Performance metrics and analysis

✅ **Python Implementation**
- Clean, modular architecture
- Hardware abstraction layers for sensors and motors
- Configurable PID parameters
- Support for multiple sensor types (camera, ultrasonic, simulated)
- Comprehensive unit tests

✅ **Documentation**
- Detailed setup instructions
- Parameter tuning guidelines
- Hardware integration guide
- Troubleshooting tips

## Project Structure

```
Freenove4WD_TargetFollowing/
├── matlab_simulation/           # MATLAB simulation files
│   ├── vehicle_model.m         # Differential drive model
│   ├── pid_controller.m        # PID controller class
│   ├── target_following_simulation.m  # Main simulation
│   ├── tune_pid_parameters.m   # Parameter tuning tool
│   └── README.md               # MATLAB documentation
├── python_implementation/       # Python implementation
│   ├── src/                    # Source code
│   │   ├── pid_controller.py
│   │   ├── motor_controller.py
│   │   ├── sensor_interface.py
│   │   ├── target_follower.py
│   │   └── config.py
│   ├── tests/                  # Unit tests
│   ├── examples/               # Usage examples
│   ├── requirements.txt
│   └── README.md               # Python documentation
├── docs/                       # Additional documentation
└── README.md                   # This file
```

## Quick Start

### MATLAB Simulation

1. Open MATLAB (R2018b or later)
2. Navigate to `matlab_simulation/`
3. Run the simulation:
   ```matlab
   target_following_simulation
   ```

### Python Simulation

1. Install dependencies:
   ```bash
   cd python_implementation
   pip install -r requirements.txt
   ```

2. Run the basic simulation:
   ```bash
   cd examples
   python3 basic_simulation.py
   ```

### Hardware Deployment

1. Set up your Freenove 4WD hardware
2. Implement hardware-specific interfaces in `examples/hardware_example.py`
3. Configure parameters in `config.json`
4. Deploy to your robot:
   ```bash
   python3 examples/hardware_example.py
   ```

## System Architecture

### Control System

The system uses a dual-loop PID control architecture:

```
Target Detection (Sensor)
    ↓
Distance & Angle Calculation
    ↓
┌─────────────────────┐    ┌─────────────────────┐
│  Distance PID       │    │  Angle PID          │
│  Controller         │    │  Controller         │
└─────────────────────┘    └─────────────────────┘
    ↓                          ↓
Forward Speed              Turn Speed
    ↓                          ↓
        Differential Drive Mixing
                ↓
    ┌───────────────────────┐
    │   Left      Right     │
    │   Motors    Motors    │
    └───────────────────────┘
```

### Key Components

1. **PID Controllers**
   - Distance controller: Maintains desired following distance
   - Angle controller: Keeps robot oriented toward target
   - Anti-windup protection prevents integral saturation

2. **Vehicle Model**
   - Differential drive kinematics
   - Converts wheel velocities to robot motion
   - Accurate simulation of 4WD behavior

3. **Sensor Interface**
   - Abstract interface for target detection
   - Supports camera, ultrasonic, and simulated sensors
   - Returns distance and angle to target

4. **Motor Controller**
   - Hardware abstraction for motor commands
   - Velocity limiting and safety features
   - Easy adaptation to different motor drivers

## Parameter Tuning

### Default PID Parameters

**Distance Controller:**
- Kp = 50.0 (MATLAB: 0.5)
- Ki = 1.0 (MATLAB: 0.01)
- Kd = 10.0 (MATLAB: 0.1)

**Angle Controller:**
- Kp = 80.0 (MATLAB: 1.5)
- Ki = 1.0 (MATLAB: 0.01)
- Kd = 15.0 (MATLAB: 0.2)

### Tuning Process

1. Start with MATLAB simulation
2. Use `tune_pid_parameters.m` to test different parameter sets
3. Observe performance metrics and trajectories
4. Select parameters with best performance
5. Scale parameters for Python implementation
6. Fine-tune on actual hardware

### Tuning Guidelines

**If the robot oscillates:**
- Reduce proportional gain (Kp)
- Increase derivative gain (Kd)

**If the robot is too slow:**
- Increase proportional gain (Kp)

**If there's steady-state error:**
- Increase integral gain (Ki) carefully

**For smoother motion:**
- Increase derivative gain (Kd)
- Reduce control loop rate

## Hardware Requirements

### Freenove 4WD Smart Car
- Raspberry Pi or compatible board
- Motor driver (L298N or similar)
- 4 DC motors with wheels
- Power supply (7.4V recommended)

### Sensors (Choose One)
- Camera module (for vision-based tracking)
- Ultrasonic sensors (for distance-based tracking)
- IR sensors (for line/object following)

### Optional
- IMU for improved orientation estimation
- Encoders for velocity feedback
- Additional sensors for obstacle avoidance

## Software Requirements

### MATLAB
- MATLAB R2018b or later
- No additional toolboxes required

### Python
- Python 3.7 or higher
- Dependencies in `requirements.txt`
- Optional: OpenCV for camera support
- Optional: RPi.GPIO for Raspberry Pi

## Testing

### MATLAB Tests
Run the simulation scripts to verify:
```matlab
target_following_simulation  % Visual verification
tune_pid_parameters         % Parameter comparison
```

### Python Tests
Run unit tests:
```bash
cd python_implementation
python -m pytest tests/ -v
```

Run specific tests:
```bash
python -m pytest tests/test_pid_controller.py -v
python -m pytest tests/test_motor_controller.py -v
```

## Performance Metrics

The system tracks several performance metrics:

- **Average Distance Error**: How well desired distance is maintained
- **Average Angle Error**: How well robot stays oriented to target
- **Settling Time**: Time to reach stable following
- **Overshoot**: Maximum error during transient response
- **Steady-State Error**: Final tracking accuracy

### Target Performance
- Distance error < 0.1 m average
- Angle error < 10° average
- Settling time < 5 seconds
- Minimal overshoot

## Examples and Use Cases

1. **Autonomous Following**: Robot follows a person or object
2. **Formation Control**: Multiple robots following each other
3. **Target Tracking**: Following a moving target at safe distance
4. **Line Following**: Using modified sensor interface
5. **Patrol Behavior**: Following predefined waypoints

## Troubleshooting

### Common Issues

**Robot doesn't move:**
- Check motor connections and power
- Verify motor speed limits are reasonable
- Ensure target is detected

**Excessive oscillation:**
- Reduce Kp gains
- Increase Kd gains
- Lower control loop rate

**Poor tracking accuracy:**
- Increase Kp gains
- Add small Ki term
- Check sensor accuracy

**Target not detected:**
- Verify sensor initialization
- Check detection parameters
- Ensure good lighting (for camera)

See individual README files for more detailed troubleshooting.

## Documentation

- [MATLAB Simulation Guide](matlab_simulation/README.md)
- [Python Implementation Guide](python_implementation/README.md)

## Contributing

Contributions are welcome! Please feel free to:
- Report bugs or issues
- Suggest new features
- Submit pull requests
- Improve documentation

## License

This project is provided for educational and research purposes. Please check with Freenove for any hardware-related licensing.

## Acknowledgments

- Freenove for the 4WD smart car platform
- Control systems theory and PID controller design
- Open-source robotics community

## Contact

For questions, issues, or suggestions, please open an issue in the repository.

## References

- [Differential Drive Kinematics](https://en.wikipedia.org/wiki/Differential_wheeled_robot)
- [PID Controller Theory](https://en.wikipedia.org/wiki/PID_controller)
- [Freenove 4WD Documentation](https://github.com/Freenove)

---

**Happy Robot Building! 🤖**
