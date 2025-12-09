# System Architecture

This document provides a detailed overview of the target following system architecture.

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Target Following System                       │
│                                                                   │
│  ┌───────────────┐      ┌─────────────┐      ┌──────────────┐  │
│  │    Sensor     │─────▶│   Control   │─────▶│    Motor     │  │
│  │   Interface   │      │   System    │      │  Controller  │  │
│  └───────────────┘      └─────────────┘      └──────────────┘  │
│         │                      │                      │          │
│         │                      │                      │          │
│         ▼                      ▼                      ▼          │
│  Target Position          PID Control           Wheel Commands  │
│  (distance, angle)      (forward, turn)         (left, right)   │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

## Component Architecture

### 1. Sensor Interface Layer

```
┌──────────────────────────────────────────────────────────┐
│                  Sensor Interface                         │
├──────────────────────────────────────────────────────────┤
│                                                            │
│  Abstract Interface: SensorInterface                      │
│  ├─ get_target_position() → {distance, angle, detected}  │
│  ├─ is_target_detected() → bool                          │
│  └─ set_detection_timeout(timeout)                       │
│                                                            │
├──────────────────────────────────────────────────────────┤
│                                                            │
│  Implementations:                                         │
│  ├─ SimulatedSensor      (for testing)                   │
│  ├─ CameraSensor         (vision-based)                  │
│  └─ UltrasonicSensor     (distance-based)                │
│                                                            │
└──────────────────────────────────────────────────────────┘
```

**Responsibilities:**
- Detect target presence
- Calculate distance to target
- Determine angle to target
- Handle sensor failures gracefully

**Key Methods:**
- `get_target_position()`: Returns target location relative to robot
- `is_target_detected()`: Checks if target is currently visible

### 2. Control System Layer

```
┌────────────────────────────────────────────────────────────────┐
│                     Control System                              │
├────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Target Follower                                                │
│  ├─ Configuration Management                                    │
│  ├─ Control Loop (@ 20 Hz default)                             │
│  ├─ Error Calculation                                           │
│  └─ Command Generation                                          │
│                                                                  │
│  ┌─────────────────────┐      ┌─────────────────────┐         │
│  │  Distance PID       │      │   Angle PID         │         │
│  │  Controller         │      │   Controller        │         │
│  ├─────────────────────┤      ├─────────────────────┤         │
│  │ Kp = 50.0          │      │ Kp = 80.0          │         │
│  │ Ki = 1.0           │      │ Ki = 1.0           │         │
│  │ Kd = 10.0          │      │ Kd = 15.0          │         │
│  │                     │      │                     │         │
│  │ Input: dist_error  │      │ Input: angle_error │         │
│  │ Output: v_forward  │      │ Output: v_turn     │         │
│  └─────────────────────┘      └─────────────────────┘         │
│           │                              │                      │
│           └──────────────┬───────────────┘                      │
│                          │                                      │
│                          ▼                                      │
│              Differential Drive Mixer                           │
│              v_left = v_forward - v_turn                        │
│              v_right = v_forward + v_turn                       │
│                                                                  │
└────────────────────────────────────────────────────────────────┘
```

**Responsibilities:**
- Maintain control loop timing
- Calculate position errors
- Compute PID control outputs
- Mix control signals for differential drive
- Handle edge cases (target lost, too close, etc.)

**Key Components:**
- `TargetFollower`: Main control orchestrator
- `PIDController`: Generic PID implementation
- Control loop: Runs at configurable frequency (default 20 Hz)

### 3. Motor Controller Layer

```
┌─────────────────────────────────────────────────────────┐
│                  Motor Controller                        │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  Abstract Interface: MotorController                     │
│  ├─ set_motor_speeds(left, right)                       │
│  ├─ stop()                                               │
│  ├─ move_forward(speed)                                  │
│  ├─ move_backward(speed)                                 │
│  ├─ turn_left(speed)                                     │
│  └─ turn_right(speed)                                    │
│                                                           │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  Features:                                               │
│  ├─ Speed limiting                                       │
│  ├─ Direction control                                    │
│  ├─ Emergency stop                                       │
│  └─ Command validation                                   │
│                                                           │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  Implementations:                                        │
│  ├─ SimulatedMotorController  (for testing)             │
│  └─ HardwareMotorController   (hardware-specific)       │
│                                                           │
└─────────────────────────────────────────────────────────┘
```

**Responsibilities:**
- Convert velocity commands to motor signals
- Enforce speed limits
- Provide high-level motion primitives
- Handle hardware communication

### 4. Configuration Layer

```
┌──────────────────────────────────────────────┐
│         Configuration Manager                 │
├──────────────────────────────────────────────┤
│                                                │
│  Configuration Storage (JSON)                 │
│  ├─ Control parameters                        │
│  ├─ PID gains                                 │
│  ├─ Physical constants                        │
│  └─ Sensor settings                           │
│                                                │
│  Methods:                                     │
│  ├─ load(file)                                │
│  ├─ save(file)                                │
│  ├─ get(key)                                  │
│  ├─ set(key, value)                           │
│  └─ reset_to_default()                        │
│                                                │
└──────────────────────────────────────────────┘
```

**Responsibilities:**
- Store and retrieve configuration
- Provide default values
- Validate configuration parameters
- Enable runtime parameter updates

## Data Flow

### Main Control Loop

```
1. Sensor Reading
   └─▶ get_target_position()
        └─▶ {distance, angle, detected}

2. Error Calculation
   └─▶ distance_error = measured_distance - desired_distance
   └─▶ angle_error = angle_to_target - robot_orientation

3. PID Control
   └─▶ v_forward = distance_PID.compute(distance_error, dt)
   └─▶ v_turn = angle_PID.compute(angle_error, dt)

4. Command Mixing
   └─▶ v_left = v_forward - v_turn
   └─▶ v_right = v_forward + v_turn

5. Motor Commands
   └─▶ motor_controller.set_motor_speeds(v_left, v_right)

6. Wait for next cycle
   └─▶ sleep(dt)
   └─▶ Go to step 1
```

### Timing Diagram

```
Time:  0ms    50ms   100ms  150ms  200ms  250ms  300ms
       │      │      │      │      │      │      │
Loop:  ├──────┼──────┼──────┼──────┼──────┼──────┤
       │      │      │      │      │      │      │
Sense: ●      ●      ●      ●      ●      ●      ●
       │      │      │      │      │      │      │
Compute:└──●   └──●   └──●   └──●   └──●   └──●   └──●
          │      │      │      │      │      │      │
Motor:    └───●  └───●  └───●  └───●  └───●  └───●  └───●

Control Loop Rate: 20 Hz (50ms period)
```

## State Machine

```
┌──────────────┐
│ INITIALIZED  │
└──────┬───────┘
       │
       │ start()
       ▼
┌──────────────┐      Target Lost      ┌──────────────┐
│   TRACKING   │◀────────────────────▶│  SEARCHING   │
└──────┬───────┘      Target Found     └──────────────┘
       │
       │ Too Close
       ▼
┌──────────────┐
│  BACKING UP  │
└──────┬───────┘
       │
       │ Safe Distance
       ▼
┌──────────────┐
│   TRACKING   │
└──────────────┘
```

**States:**
1. **INITIALIZED**: System ready, waiting for start command
2. **TRACKING**: Actively following detected target
3. **SEARCHING**: No target detected, rotating to find
4. **BACKING UP**: Too close to target, moving backward

## Class Diagram

```
┌─────────────────────┐
│  TargetFollower     │
├─────────────────────┤
│ - sensor            │
│ - motor_controller  │
│ - distance_pid      │
│ - angle_pid         │
│ - config            │
├─────────────────────┤
│ + start()           │
│ + stop()            │
│ + set_desired_distance() │
│ - _control_step()   │
│ - _handle_no_target()    │
└──────┬──────────────┘
       │
       │ uses
       │
       ├────────────────────────────┐
       │                            │
       ▼                            ▼
┌─────────────────┐       ┌──────────────────┐
│ PIDController   │       │ SensorInterface  │
├─────────────────┤       ├──────────────────┤
│ - Kp, Ki, Kd    │       │                  │
│ - integral      │       │                  │
│ - prev_error    │       │                  │
├─────────────────┤       ├──────────────────┤
│ + compute()     │       │ + get_target_position() │
│ + reset()       │       │ + is_target_detected()  │
│ + set_gains()   │       │                  │
└─────────────────┘       └────────┬─────────┘
                                   │
                                   │ implements
                                   │
                     ┌─────────────┼─────────────┐
                     │             │             │
                     ▼             ▼             ▼
              ┌──────────┐  ┌──────────┐  ┌──────────┐
              │Simulated │  │ Camera   │  │Ultrasonic│
              │ Sensor   │  │ Sensor   │  │ Sensor   │
              └──────────┘  └──────────┘  └──────────┘
```

## File Organization

```
python_implementation/src/
├── __init__.py              # Package initialization
├── pid_controller.py        # PID implementation (200 lines)
├── motor_controller.py      # Motor abstraction (200 lines)
├── sensor_interface.py      # Sensor abstraction (250 lines)
├── target_follower.py       # Main control loop (250 lines)
└── config.py                # Configuration management (150 lines)

Total: ~1050 lines of core implementation
```

## Communication Protocols

### Between Components

```
SensorInterface → TargetFollower
    Message: {'distance': float, 'angle': float, 'detected': bool}
    Frequency: 20 Hz

TargetFollower → MotorController
    Message: (left_speed: float, right_speed: float)
    Frequency: 20 Hz

ConfigManager → All Components
    Message: Configuration parameters (dict)
    Frequency: On startup and updates
```

## Performance Characteristics

### Latency Budget (50ms control loop)

```
Component               Typical Time    Max Time
─────────────────────────────────────────────────
Sensor Reading          5-10ms         20ms
Error Calculation       <1ms           1ms
PID Computation         <1ms           1ms
Motor Command           1-5ms          10ms
Communication Overhead  2-5ms          10ms
─────────────────────────────────────────────────
Total                   ~15ms          42ms
Margin                  35ms           8ms
```

### Memory Usage

```
Component               Memory
──────────────────────────────
PID Controllers (2x)    ~1 KB
Sensor Interface        ~2 KB
Motor Controller        ~1 KB
Configuration           ~5 KB
Command History         ~10 KB
──────────────────────────────
Total                   ~20 KB
```

## Extensibility Points

### Adding New Sensors

```python
class MyCustomSensor(SensorInterface):
    def get_target_position(self):
        # Your detection logic here
        return {'distance': d, 'angle': a, 'detected': True}
```

### Adding New Controllers

```python
class MyAdvancedController:
    def __init__(self):
        # Use LQR, MPC, or other advanced control
        pass
    
    def compute(self, state, target):
        # Your control logic
        return control_output
```

### Adding Logging

```python
class LoggingTargetFollower(TargetFollower):
    def _control_step(self, target_info, dt):
        # Log before
        super()._control_step(target_info, dt)
        # Log after
```

## Testing Architecture

```
Unit Tests
├── test_pid_controller.py      # PID logic
├── test_motor_controller.py    # Motor commands
├── test_sensor_interface.py    # Sensor data
└── test_target_follower.py     # Integration

Integration Tests
├── test_full_system.py         # End-to-end
└── test_hardware_interface.py  # Hardware mocks

Simulation Tests
├── matlab_simulation/          # MATLAB validation
└── python_simulation/          # Python validation
```

## Deployment Configurations

### Development
- Simulated sensors
- Debug logging enabled
- Low control frequency (5-10 Hz)

### Testing
- Real sensors
- Performance logging
- Normal frequency (20 Hz)

### Production
- Optimized sensors
- Error logging only
- High frequency (20-50 Hz)

---

This architecture provides a robust, modular, and extensible foundation for target following applications.
