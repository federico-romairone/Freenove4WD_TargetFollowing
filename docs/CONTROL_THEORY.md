# Control Theory and System Design

This document explains the theoretical foundations and design decisions behind the target following control system.

## Table of Contents

1. [System Overview](#system-overview)
2. [Vehicle Kinematics](#vehicle-kinematics)
3. [PID Control](#pid-control)
4. [Dual-Loop Control Architecture](#dual-loop-control-architecture)
5. [Stability Analysis](#stability-analysis)
6. [Performance Tuning](#performance-tuning)

## System Overview

The target following system implements a feedback control loop where:
- **Input**: Target position (distance and angle)
- **Output**: Robot motion (wheel velocities)
- **Objective**: Maintain desired distance while tracking target

### Control Flow

```
Target Detection → Error Calculation → PID Controllers → Motor Commands → Robot Motion
        ↑                                                                      ↓
        └──────────────────────── Feedback Loop ──────────────────────────────┘
```

## Vehicle Kinematics

### Differential Drive Model

The Freenove 4WD uses differential drive kinematics, where motion is controlled by varying the speeds of left and right wheels.

#### Forward Kinematics

Given left and right wheel velocities (v_L, v_R):

```
v = (v_L + v_R) / 2           # Linear velocity
ω = (v_R - v_L) / L           # Angular velocity
```

Where:
- v: Forward velocity of robot center
- ω: Angular velocity (turning rate)
- L: Wheel base (distance between wheels)

#### Position Update

Robot position evolves according to:

```
x(t+Δt) = x(t) + v·cos(θ)·Δt
y(t+Δt) = y(t) + v·sin(θ)·Δt
θ(t+Δt) = θ(t) + ω·Δt
```

Where:
- (x, y): Robot position
- θ: Robot orientation
- Δt: Time step

### Inverse Kinematics

To achieve desired forward velocity (v) and turning rate (ω):

```
v_L = v - (ω·L)/2
v_R = v + (ω·L)/2
```

## PID Control

### PID Equation

A PID controller computes control output based on error signal:

```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de(t)/dt
```

Where:
- e(t): Error signal (setpoint - measurement)
- Kp: Proportional gain
- Ki: Integral gain
- Kd: Derivative gain

### Terms Explained

#### Proportional Term (P)

```
P = Kp · e(t)
```

- **Effect**: Output proportional to current error
- **Benefit**: Fast response to errors
- **Drawback**: May cause steady-state error and oscillations

#### Integral Term (I)

```
I = Ki · ∫e(τ)dτ
```

- **Effect**: Accumulates past errors
- **Benefit**: Eliminates steady-state error
- **Drawback**: Can cause overshoot and windup

#### Derivative Term (D)

```
D = Kd · de(t)/dt
```

- **Effect**: Responds to rate of error change
- **Benefit**: Reduces overshoot, improves stability
- **Drawback**: Amplifies noise

### Anti-Windup

Integral windup occurs when integral term accumulates excessively during saturation.

**Solution**: Clamp integral term when output saturates

```python
if output > limit:
    integral = min(integral, max_integral)
```

## Dual-Loop Control Architecture

### Distance Controller

**Purpose**: Maintain desired following distance

**Error Signal**:
```
e_dist = measured_distance - desired_distance
```

**Output**: Forward velocity command

**Behavior**:
- Positive error → Move forward
- Negative error → Move backward
- Zero error → Maintain position

### Angle Controller

**Purpose**: Keep robot oriented toward target

**Error Signal**:
```
e_angle = atan2(target_y - robot_y, target_x - robot_x) - robot_theta
```

Note: Angle normalized to [-π, π]

**Output**: Turning velocity command

**Behavior**:
- Positive error → Turn left
- Negative error → Turn right
- Zero error → Move straight

### Combining Controllers

Final wheel velocities:

```
v_forward = distance_PID(e_dist)
v_turn = angle_PID(e_angle)

v_left = v_forward - v_turn
v_right = v_forward + v_turn
```

## Stability Analysis

### Lyapunov Stability

For the system to be stable, we need:

1. **Bounded output**: Control signals must remain within limits
2. **Convergence**: Errors must decrease over time
3. **No oscillations**: System should settle smoothly

### Critical Parameters

#### Proportional Gain (Kp)

- **Too high**: System becomes unstable, oscillates
- **Too low**: Slow response, large steady-state error
- **Optimal**: Fast response without oscillation

#### Derivative Gain (Kd)

- **Too high**: Sensitive to noise, jittery motion
- **Too low**: Poor damping, overshooting
- **Optimal**: Smooth motion, quick settling

#### Integral Gain (Ki)

- **Too high**: Overshoot, instability
- **Too low**: Persistent steady-state error
- **Optimal**: Eliminates error without overshoot

### Stability Margins

For differential drive robot:

**Phase Margin**: > 45° (recommended)
**Gain Margin**: > 6 dB (recommended)

These ensure robust performance under:
- Model uncertainties
- Sensor noise
- Environmental disturbances

## Performance Tuning

### Tuning Methods

#### 1. Ziegler-Nichols Method

1. Set Ki = Kd = 0
2. Increase Kp until sustained oscillation
3. Record critical gain (Kc) and period (Tc)
4. Calculate:
   ```
   Kp = 0.6 × Kc
   Ki = 2 × Kp / Tc
   Kd = Kp × Tc / 8
   ```

#### 2. Manual Tuning

Sequential tuning process:

1. **Start with P-only**:
   - Set Ki = Kd = 0
   - Increase Kp until slight oscillation
   - Reduce Kp by 25-50%

2. **Add D term**:
   - Increase Kd to reduce oscillation
   - Stop when motion is smooth

3. **Add I term** (if needed):
   - Increase Ki slowly
   - Stop when steady-state error eliminated

#### 3. Simulation-Based Tuning

1. Test in MATLAB simulation
2. Try multiple parameter sets
3. Evaluate metrics:
   - Rise time
   - Settling time
   - Overshoot
   - Steady-state error
4. Select best parameters
5. Scale for hardware

### Parameter Scaling

MATLAB uses velocities in m/s, Python uses motor speeds (0-100).

**Scaling factor**:
```
scale = max_motor_speed / max_velocity
```

**Example**:
```
Kp_matlab = 0.5
max_motor_speed = 80
max_velocity = 0.3

Kp_python = 0.5 × (80 / 0.3) ≈ 133
```

### Performance Metrics

#### Time-Domain Metrics

1. **Rise Time (Tr)**: Time to reach 90% of setpoint
   - Target: < 2 seconds

2. **Settling Time (Ts)**: Time to stay within 5% of setpoint
   - Target: < 5 seconds

3. **Overshoot (Mp)**: Maximum error beyond setpoint
   - Target: < 20%

4. **Steady-State Error (ess)**: Final error
   - Target: < 0.05 m (distance), < 5° (angle)

#### Frequency-Domain Metrics

1. **Bandwidth**: Frequency range of effective control
   - Target: > 2 Hz

2. **Phase Margin**: Stability margin in phase
   - Target: > 45°

3. **Gain Margin**: Stability margin in gain
   - Target: > 6 dB

### Practical Tuning Tips

1. **For faster response**: Increase Kp
2. **For less overshoot**: Increase Kd, decrease Kp
3. **For better accuracy**: Increase Ki (carefully)
4. **For smoother motion**: Increase Kd
5. **For robustness**: Maintain good stability margins

### Common Issues and Solutions

| Issue | Likely Cause | Solution |
|-------|--------------|----------|
| Oscillation | Kp too high | Reduce Kp, increase Kd |
| Slow response | Kp too low | Increase Kp |
| Steady-state error | No integral action | Increase Ki |
| Overshoot | Kp too high or Kd too low | Reduce Kp, increase Kd |
| Noise sensitivity | Kd too high | Reduce Kd, filter sensor |
| Windup | Ki too high | Reduce Ki, implement anti-windup |

## Advanced Topics

### Adaptive Control

For varying conditions, consider:
- Gain scheduling based on distance
- Self-tuning algorithms
- Model reference adaptive control (MRAC)

### Feedforward Control

Add feedforward term for better tracking:
```
u = PID(error) + feedforward(target_velocity)
```

### State-Space Control

For advanced control:
- Linear Quadratic Regulator (LQR)
- Model Predictive Control (MPC)
- H-infinity control

### Disturbance Rejection

Improve robustness against:
- Terrain variations
- Wind disturbances
- Sensor noise

Techniques:
- Disturbance observer
- Robust PID tuning
- Kalman filtering

## References

1. Åström, K. J., & Murray, R. M. (2008). *Feedback Systems: An Introduction for Scientists and Engineers*
2. Siegwart, R., & Nourbakhsh, I. R. (2004). *Introduction to Autonomous Mobile Robots*
3. Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2015). *Feedback Control of Dynamic Systems*
4. Ogata, K. (2010). *Modern Control Engineering*

## Glossary

- **Setpoint**: Desired value of controlled variable
- **Error**: Difference between setpoint and measurement
- **Control Signal**: Output of controller to actuators
- **Feedback**: Measurement used to compute error
- **Feedforward**: Direct path from input to output
- **Saturation**: Limiting of control signal to physical bounds
- **Windup**: Excessive integral accumulation during saturation
- **Stability**: Property that errors remain bounded
- **Robustness**: Performance despite uncertainties

---

For practical implementation details, see the main README and code documentation.
