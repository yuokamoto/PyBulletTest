# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a PyBullet-based robotics simulation project focused on creating scalable simulations. The project aims to simulate 100 robots at 10x speed while maintaining collision detection capabilities.

## Common Commands

### Setup (with virtual environment - recommended)
```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
source venv/bin/activate  # Linux/Mac
# venv\Scripts\activate   # Windows

# Install dependencies
pip install -r requirements.txt
```

### Setup (direct installation)
```bash
pip install -r requirements.txt
```

### Running Simulations
```bash
# Configurable demo (recommended)
python configurable_demo.py --robots 50 --speed 5
python configurable_demo.py --help  # Show all options

# Main simulation (100 robots, fixed)
python multi_robot_simulation.py

# Simple test (10 robots)
python simple_example.py
```

### Testing
Since this is a simulation project, testing is primarily done through visual verification and performance monitoring during execution.

## Code Architecture

### Main Components

- **MultiRobotSimulation class** (`multi_robot_simulation.py:24`): Core simulation manager
  - Handles PyBullet initialization and configuration
  - Manages robot spawning and lifecycle
  - Controls simulation speed and physics settings
  
- **Robot class** (`multi_robot_simulation.py:13`): Individual robot representation  
  - Stores robot state and command interfaces
  - Provides velocity and joint control methods
  
### Key Design Patterns

- **Performance-first approach**: Physics calculations are minimal to achieve 10x speed
- **Collision detection**: Uses PyBullet's built-in contact point detection
- **Random movement**: Each robot receives random velocity commands periodically
- **Scalable architecture**: Designed to handle 100+ robots efficiently

## Project Requirements (from memo.txt)

**Primary Objectives:**
- Scalable simulation using PyBullet  
- Support for 100 robots at 10x simulation speed
- Physics calculations can be omitted for performance
- Collision detection is required

**Interface Requirements:**
- Robot body velocity commands
- Joint velocity and position commands  
- Future compatibility with https://github.com/ros-simulation/simulation_interfaces
- Object-to-object connections

**Current Implementation:**
- ✅ 100 robots simulation implemented
- ✅ 10x speed simulation working
- ✅ Collision detection functional
- ✅ Random movement example created

**Future Features:**
- URDF-based robot generation
- ROS 2 integration for robot control

## Development Notes

- Primary language: Python (PyBullet's native API)
- Simulation uses simple box-shaped robots for performance
- No real-time simulation - runs as fast as possible
- Japanese documentation in memo.txt
- Focus on performance over visual fidelity