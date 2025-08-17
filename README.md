# PyBullet Multi-Robot Simulation

A PyBullet-based project for simulating 100 robots at 10x speed with collision detection capabilities.

## Features

- ✅ Simultaneous simulation of 100 robots
- ✅ 10x speed simulation
- ✅ URDF-based robot generation (arm robots and mobile robots)
- ✅ Collision detection functionality
- ✅ Advanced URDF caching system
- ✅ External data monitor window
- ✅ Performance optimization (physics disable support)
- ✅ Real-time robot control (joint control & velocity control)

## Setup

### Virtual Environment Setup (Recommended)

We recommend using a venv virtual environment to isolate project dependencies.

```bash
# 1. Create virtual environment
python -m venv venv

# 2. Activate virtual environment
# For Linux/Mac:
source venv/bin/activate

# For Windows:
# venv\Scripts\activate

# 3. Install required packages
pip install -r requirements.txt
```

### Deactivating Virtual Environment

When you're done working, you can deactivate the virtual environment:

```bash
deactivate
```

## Usage

### Basic Usage

```bash
# Default: 10 robots, 1x speed, 30 seconds
python configurable_demo.py

# 50 robots at 5x speed
python configurable_demo.py --robots 50 --speed 5

# 100 robots, 10x speed, 60 seconds
python configurable_demo.py --robots 100 --speed 10 --duration 60

# With external data monitor window
python configurable_demo.py --robots 20 --speed 2 --monitor

# GUI disabled for better performance (200 robots, 20x speed)
python configurable_demo.py --robots 200 --speed 20 --no-gui --verbose

# Show help
python configurable_demo.py --help
```

### Performance Optimization Settings

```bash
# Maximum performance: GUI disabled, physics disabled, external monitor
python configurable_demo.py --robots 100 --speed 10 --no-gui --monitor --verbose

# High frequency simulation (1000Hz, low speed)
python configurable_demo.py --robots 10 --timestep 0.001 --speed 0.5

# Console monitoring (no GUI required)
python configurable_demo.py --robots 50 --speed 5 --monitor --console-monitor
```

### Parameters

**Basic Settings:**
- `--robots, -r`: Number of robots (default: 10)
- `--speed, -s`: Realtime factor multiplier (default: 1.0)
- `--duration, -d`: Simulation duration in seconds (default: 30)
- `--timestep, -t`: Simulation timestep in seconds (default: 0.004167, 240Hz)

**Display & Control:**
- `--gui` / `--no-gui`: Enable/disable GUI (default: enabled)
- `--physics` / `--no-physics`: Enable/disable physics (default: disabled)
- `--monitor`: Enable data monitor (file-based by default)
- `--console-monitor`: Use console-based monitor (no GUI, text only)
- `--verbose, -v`: Enable verbose output

## File Structure

- `configurable_demo.py` - **Main simulation** (configurable, URDF-compatible)
- `data_monitor.py` - External data monitor window
- `robots/` - URDF robot definition files
  - `arm_robot.urdf` - 4-DOF arm robot
  - `mobile_robot.urdf` - Mobile robot (forward/backward movement + yaw rotation)
- `requirements.txt` - Required Python packages
- `memo.txt` - Project specifications (Japanese)
- `CLAUDE.md` - Project details and command reference
- `README_ja.md` - Japanese version of this README

## Robot Specifications

### Arm Robot
- **Fixed Base**: Fixed to floor (z=0)
- **Control Method**: Joint position control
- **Degrees of Freedom**: 4 DOF (shoulder, elbow, wrist, end_effector)
- **Movement**: Random joint angle targets

### Mobile Robot
- **Movement Range**: Ground height 0.3m
- **Control Method**: Direct velocity control
- **Movement Constraints**: Forward/backward movement and yaw rotation only in robot coordinate system (no lateral movement)
- **Behavior**: Forward/backward movement in robot's facing direction with random yaw angular velocity
- **Coordinate System**: Movement along robot-fixed coordinate system X-axis (forward direction)

## Performance Specifications

### Optimization Techniques
- **URDF Caching System**: Pre-caching of file contents, joint information, and visual data
- **Batch Processing**: Simultaneous generation of multiple robots
- **Rendering Control**: Disabled rendering during generation
- **PyBullet Engine Optimization**: Non-physics mode support

### Achieved Performance
- **Target**: 100 robots @ 10x speed
- **Generation Speed**: 500+ robots/sec
- **Execution Speed**: Maintains target speed
- **External Monitor**: No performance impact

## External Data Monitor

Real-time data monitoring is available with the `--monitor` flag:

- **Displayed Data**: Simulation time, real time, speed factor, robot count, collision count
- **Update Frequency**: Dynamic based on timestep (0.1-1.0 second intervals)
- **Performance**: No impact on main simulation
- **GUI Control**: Automatically disables on-screen text display when monitor is enabled
- **Console Option**: JSON formatted output with `--console-monitor`

## Troubleshooting

### Performance Issues
- Disable GUI: `--no-gui`
- Disable physics: `--no-physics` (default)
- Use external monitor: `--monitor` (disables on-screen text)
- Use console monitor: `--monitor --console-monitor` (no GUI required)

### Camera Controls (GUI Mode)
- **Zoom**: Mouse wheel
- **Rotate**: Right-click + drag
- **Pan**: Middle-click + drag

## Technical Specifications

### Dependencies
- Python 3.8+
- PyBullet
- NumPy
- tkinter (for external monitor)

### Architecture
- **Object-Oriented Design**: ConfigurableRobotSimulation class
- **Caching System**: Memory efficiency and performance optimization
- **Multi-Process Support**: External monitor (tkinter)
- **Extensibility**: Easy addition of new URDF robots

## Language Support

- **English**: README.md (this file)
- **Japanese**: README_ja.md