#!/usr/bin/env python3
"""
Simple example: 10 robots moving randomly for quick testing
"""

import pybullet as p
import pybullet_data
import numpy as np
import random
import time

def create_robot(position, color=None):
    """Create a simple box robot"""
    if color is None:
        color = [random.random(), random.random(), random.random(), 1.0]
    
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.2, 0.1])
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.2, 0.1], rgbaColor=color)
    
    robot = p.createMultiBody(baseMass=1.0,
                            baseCollisionShapeIndex=collision_shape,
                            baseVisualShapeIndex=visual_shape,
                            basePosition=position)
    return robot

def main():
    # Initialize PyBullet
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Setup environment
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)
    p.setRealTimeSimulation(0)
    
    # Load ground
    plane = p.loadURDF("plane.urdf")
    
    # Create 10 robots for testing
    robots = []
    for i in range(10):
        x = random.uniform(-5, 5)
        y = random.uniform(-5, 5)
        robot = create_robot([x, y, 1.0])
        robots.append(robot)
    
    print("Simple example: 10 robots created")
    print("Press Ctrl+C to stop")
    
    try:
        step = 0
        while True:
            # Move robots randomly every 60 steps
            if step % 60 == 0:
                for robot in robots:
                    vel = [random.uniform(-1, 1), random.uniform(-1, 1), 0]
                    ang_vel = [0, 0, random.uniform(-0.5, 0.5)]
                    p.resetBaseVelocity(robot, vel, ang_vel)
            
            # Step simulation 5x speed
            for _ in range(5):
                p.stepSimulation()
            
            step += 1
            time.sleep(1./240./5)
            
    except KeyboardInterrupt:
        print("\nStopping simulation...")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()