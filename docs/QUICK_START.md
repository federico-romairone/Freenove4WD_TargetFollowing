# Quick Start Guide

Get up and running with the Freenove 4WD Target Following System in minutes!

## Prerequisites

- **For MATLAB Simulation**: MATLAB R2018b or later
- **For Python**: Python 3.7 or higher
- **For Hardware**: Freenove 4WD Smart Car with sensors

## 5-Minute Quick Start

### Option 1: MATLAB Simulation (Recommended First Step)

1. **Open MATLAB**
   ```
   Open MATLAB and navigate to the project folder
   ```

2. **Run the simulation**
   ```matlab
   cd matlab_simulation
   target_following_simulation
   ```

3. **Watch the magic happen!**
   - You'll see the robot following a moving target
   - Real-time plots show trajectories and errors
   - Results are saved automatically

**That's it!** You now have a working simulation.

### Option 2: Python Simulation

1. **Install dependencies**
   ```bash
   cd python_implementation
   pip install -r requirements.txt
   ```

2. **Run the simulation**
   ```bash
   cd examples
   python3 basic_simulation.py
   ```

3. **Observe the output**
   - Console shows real-time position and tracking status
   - Simulation runs for 20 seconds by default
   - Control-C to stop early

**Done!** Your Python simulation is working.

## Next Steps

### Step 1: Understand the System

Read the [Main README](../README.md) to understand:
- System architecture
- Control theory basics
- Component overview

### Step 2: Tune Parameters (MATLAB)

```matlab
cd matlab_simulation
tune_pid_parameters
```

This helps you find optimal PID gains by:
- Testing multiple parameter sets
- Comparing performance visually
- Showing quantitative metrics

### Step 3: Test Different Scenarios

Edit the target trajectory in `target_following_simulation.m`:

```matlab
% Circular trajectory (default)
target_x = 2 + 1.5 * cos(0.3 * t);
target_y = 2 + 1.5 * sin(0.3 * t);

% OR: Straight line
target_x = 0.1 * t;
target_y = 2;

% OR: Figure-8
target_x = 2 + 2 * sin(0.3 * t);
target_y = 2 + sin(0.6 * t);
```

### Step 4: Deploy to Hardware

1. **Prepare configuration**
   ```bash
   cd python_implementation
   cp config.json.example config.json
   # Edit config.json with your parameters
   ```

2. **Implement hardware interfaces**
   - Edit `examples/hardware_example.py`
   - Customize motor controller for your setup
   - Implement sensor interface for your camera/sensors

3. **Test on hardware**
   ```bash
   python3 examples/hardware_example.py
   ```

## Common Workflows

### Workflow 1: Parameter Tuning

```
MATLAB Simulation → Tune Parameters → Test in MATLAB → Deploy to Python → Test on Hardware
```

### Workflow 2: Algorithm Development

```
Modify MATLAB Code → Test in Simulation → Validate Results → Port to Python → Deploy
```

### Workflow 3: Hardware Integration

```
Define Hardware Interface → Create Python Class → Test with Simulated Sensor → Connect Real Hardware → Test
```

## Directory Navigation

```
Freenove4WD_TargetFollowing/
├── matlab_simulation/          ← Start here for simulation
│   ├── target_following_simulation.m  ← Run this first
│   └── tune_pid_parameters.m          ← Run this second
│
├── python_implementation/
│   ├── examples/
│   │   ├── basic_simulation.py        ← Python simulation
│   │   └── hardware_example.py        ← Hardware template
│   └── src/                            ← Core library code
│
└── docs/
    ├── CONTROL_THEORY.md              ← Learn the theory
    └── QUICK_START.md                 ← You are here!
```

## Troubleshooting Quick Fixes

### MATLAB Issues

**Problem**: MATLAB can't find files
```matlab
cd /path/to/Freenove4WD_TargetFollowing/matlab_simulation
addpath(pwd)
```

**Problem**: Figure doesn't display
```matlab
close all
target_following_simulation
```

### Python Issues

**Problem**: Module not found
```bash
pip install -r requirements.txt
python3 -c "import sys; print(sys.path)"
```

**Problem**: Permission denied
```bash
chmod +x examples/basic_simulation.py
python3 examples/basic_simulation.py
```

**Problem**: Tests fail
```bash
cd python_implementation
python3 -m pytest tests/ -v
```

## Quick Commands Reference

### MATLAB
```matlab
% Run main simulation
target_following_simulation

% Tune parameters
tune_pid_parameters

% Clear and restart
clear all; close all; clc;

% Save current figure
saveas(gcf, 'my_results.png')
```

### Python
```bash
# Install dependencies
pip install -r requirements.txt

# Run tests
python3 -m pytest tests/ -v

# Run simulation
python3 examples/basic_simulation.py

# Run specific test
python3 tests/test_pid_controller.py
```

### Git
```bash
# Clone repository
git clone <repository-url>
cd Freenove4WD_TargetFollowing

# Check status
git status

# View changes
git diff
```

## Learning Path

### Beginner Track
1. ✅ Run MATLAB simulation
2. ✅ Run Python simulation
3. 📖 Read main README
4. 🔧 Modify parameters
5. 📊 Observe results

### Intermediate Track
1. ✅ Complete Beginner Track
2. 📖 Read Control Theory doc
3. 🔧 Use parameter tuning tool
4. 💻 Modify simulation code
5. 🧪 Run Python tests

### Advanced Track
1. ✅ Complete Intermediate Track
2. 🛠️ Implement hardware interfaces
3. 📷 Set up camera detection
4. 🤖 Deploy to robot
5. 🎯 Optimize for your application

## Getting Help

1. **Check the documentation**
   - Main [README](../README.md)
   - [MATLAB README](../matlab_simulation/README.md)
   - [Python README](../python_implementation/README.md)
   - [Control Theory](CONTROL_THEORY.md)

2. **Run the examples**
   - All examples include comments
   - Start simple, then customize

3. **Check the tests**
   - Unit tests show expected behavior
   - Use as reference for your code

4. **Open an issue**
   - Describe your problem
   - Include error messages
   - Share your configuration

## Tips for Success

1. **Start with simulation** - Don't jump straight to hardware
2. **Tune incrementally** - Small changes, test often
3. **Use version control** - Commit working states
4. **Document changes** - Comment your modifications
5. **Test frequently** - Catch issues early

## What's Next?

After completing this quick start:

1. **Experiment with parameters** - See how they affect behavior
2. **Try different trajectories** - Test various scenarios
3. **Add features** - Obstacle avoidance, multiple targets, etc.
4. **Optimize performance** - Fine-tune for your specific needs
5. **Share your results** - Contribute back to the community

## Resources

- **Repository**: Full code and documentation
- **Issues**: Report bugs or ask questions
- **Discussions**: Share ideas and get help
- **Wiki**: Additional tutorials and guides (if available)

---

**Ready to build something awesome? Let's go! 🚀**
