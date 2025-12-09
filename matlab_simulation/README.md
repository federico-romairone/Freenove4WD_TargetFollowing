# MATLAB Simulation for Freenove 4WD Target Following

This directory contains MATLAB simulation files for developing and testing the closed-loop control system before deploying to hardware.

## Overview

The MATLAB simulation provides:
- Differential drive kinematic model
- PID controller implementation
- Target following simulation with visualization
- Parameter tuning tools

## Files

- `vehicle_model.m`: Differential drive kinematic model
- `pid_controller.m`: PID controller class
- `target_following_simulation.m`: Main simulation script with visualization
- `tune_pid_parameters.m`: Interactive parameter tuning tool

## Requirements

- MATLAB R2018b or later
- No additional toolboxes required (uses base MATLAB)

## Running the Simulation

### Basic Simulation

1. Open MATLAB
2. Navigate to the `matlab_simulation` directory
3. Run the main simulation:

```matlab
target_following_simulation
```

This will:
- Simulate a robot following a moving target
- Display real-time trajectory plots
- Show distance and angle errors
- Save results as `target_following_results.png`

### Parameter Tuning

To optimize PID parameters:

```matlab
tune_pid_parameters
```

This script:
- Tests multiple parameter sets
- Compares performance metrics
- Visualizes results side-by-side
- Helps identify optimal gains

## Simulation Parameters

### Vehicle Parameters
```matlab
wheel_base = 0.15;      % Distance between wheels (m)
max_velocity = 0.3;     % Maximum wheel velocity (m/s)
```

### Controller Parameters (Default)
```matlab
% Distance controller
Kp_dist = 0.5;
Ki_dist = 0.01;
Kd_dist = 0.1;

% Angle controller
Kp_angle = 1.5;
Ki_angle = 0.01;
Kd_angle = 0.2;
```

### Control Parameters
```matlab
desired_distance = 0.5;  % Desired following distance (m)
dt = 0.05;               % Simulation time step (s)
t_end = 30;              % Simulation duration (s)
```

## Customizing the Simulation

### Changing the Target Trajectory

Edit the target position in `target_following_simulation.m`:

```matlab
% Example: Circular trajectory
target_x = 2 + 1.5 * cos(0.3 * t);
target_y = 2 + 1.5 * sin(0.3 * t);

% Example: Straight line
% target_x = 0.1 * t;
% target_y = 2;

% Example: Figure-8
% target_x = 2 + 2 * sin(0.3 * t);
% target_y = 2 + sin(0.6 * t);
```

### Adjusting Initial Conditions

```matlab
% Robot starting position
robot_x = 0;
robot_y = 0;
robot_theta = 0;  % radians

% Target starting position
target_x = 2;
target_y = 2;
```

### Modifying PID Parameters

```matlab
% More aggressive control
Kp_dist = 0.7;
Kd_dist = 0.15;

% More conservative control
Kp_dist = 0.3;
Kd_dist = 0.05;
```

## Understanding the Results

### Trajectory Plot
- Red dashed line: Target trajectory
- Blue solid line: Robot trajectory
- Red circle: Current target position
- Blue square: Current robot position
- Blue arrow: Robot orientation

### Distance Error Plot
- Shows how well the robot maintains desired distance
- Positive error: Robot is too far
- Negative error: Robot is too close
- Goal: Error converges to near zero

### Angle Error Plot
- Shows how well the robot points toward target
- Goal: Small oscillations around zero
- Large oscillations indicate tuning needed

## Performance Metrics

The simulation outputs:
- **Average distance error**: Mean absolute distance error
- **Max distance error**: Worst-case distance error
- **Final distance error**: Error at end of simulation
- **Average angle error**: Mean absolute angle error (degrees)
- **Max angle error**: Worst-case angle error
- **Final angle error**: Error at end of simulation

Good performance typically shows:
- Average distance error < 0.1 m
- Average angle error < 10 degrees
- Quick settling time (< 5 seconds)

## Parameter Tuning Guidelines

### Distance Controller

**Kp (Proportional Gain)**
- Controls response speed to distance errors
- Too high: Oscillations, overshooting
- Too low: Slow response, large steady-state error
- Typical range: 0.3 - 0.8

**Ki (Integral Gain)**
- Eliminates steady-state error
- Too high: Windup, overshooting
- Too low: Persistent error
- Typical range: 0.005 - 0.02

**Kd (Derivative Gain)**
- Reduces oscillations, improves stability
- Too high: Sensitive to noise, jittery
- Too low: Poor damping, oscillations
- Typical range: 0.05 - 0.2

### Angle Controller

**Kp (Proportional Gain)**
- Controls turning response
- Too high: Rapid oscillations
- Too low: Poor tracking, wide turns
- Typical range: 1.0 - 2.5

**Ki (Integral Gain)**
- Corrects steady-state angle bias
- Usually kept small
- Typical range: 0.005 - 0.02

**Kd (Derivative Gain)**
- Smooths turning motion
- Too high: Jerky motion
- Too low: Overshooting in turns
- Typical range: 0.1 - 0.4

## Exporting Parameters to Python

Once you find good parameters in MATLAB:

1. Note the optimal values
2. Update `python_implementation/src/config.py` with these values
3. Scale the gains appropriately for your hardware motor speeds

Example conversion:
```matlab
% MATLAB (velocities in m/s)
Kp_dist = 0.5;

% Python (motor speeds 0-100)
% Scale by (max_motor_speed / max_velocity)
Kp_dist_python = 0.5 * (80 / 0.3) = 133
```

## Troubleshooting

### Simulation runs but robot doesn't move
- Check initial positions (robot and target should be separated)
- Verify max_velocity > 0
- Check that PID gains are not zero

### Robot oscillates excessively
- Reduce Kp gains
- Increase Kd gains
- Reduce control loop rate (increase dt)

### Robot doesn't reach target
- Increase Kp gains
- Add small Ki term to eliminate steady-state error
- Check desired_distance is reasonable

### Simulation is slow
- Reduce visualization frequency (change `mod(i, 10)` to higher value)
- Reduce t_end
- Increase dt (but maintain stability)

## Advanced Topics

### Adding Noise

To test robustness, add sensor noise:

```matlab
% Add Gaussian noise to distance measurement
distance = sqrt(dx^2 + dy^2) + 0.05 * randn();

% Add noise to angle measurement
angle_to_target = atan2(dy, dx) + 0.1 * randn();
```

### Testing Different Scenarios

```matlab
% Static target
target_x = 2;
target_y = 2;

% Accelerating target
target_x = 2 + 0.1 * t^2;
target_y = 2;

% Random walk target
persistent target_vx target_vy;
if isempty(target_vx)
    target_vx = 0;
    target_vy = 0;
end
target_vx = target_vx + 0.01 * randn();
target_vy = target_vy + 0.01 * randn();
target_x = target_x + target_vx * dt;
target_y = target_y + target_vy * dt;
```

## References

- Differential Drive Kinematics: https://en.wikipedia.org/wiki/Differential_wheeled_robot
- PID Control: https://en.wikipedia.org/wiki/PID_controller
- Freenove 4WD Documentation: Check manufacturer's specifications

## Support

For issues or questions, please refer to the main project README or open an issue in the repository.
