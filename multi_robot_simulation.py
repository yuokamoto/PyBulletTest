#!/usr/bin/env python3
"""
Multi-robot simulation using PyBullet
Scalable simulation with 100 robots at 10x speed
"""

import pybullet as p
import pybullet_data
import numpy as np
import random
import time
import math

class Robot:
    """Simple robot representation with basic movement capabilities"""
    
    def __init__(self, robot_id, position):
        self.id = robot_id
        self.position = position
        self.target_velocity = [0, 0, 0]
        self.joint_targets = []
        
    def set_velocity_command(self, linear_vel, angular_vel):
        """Set body velocity command"""
        self.target_velocity = [linear_vel[0], linear_vel[1], angular_vel]
        
    def set_joint_command(self, joint_positions, joint_velocities=None):
        """Set joint position/velocity commands"""
        self.joint_targets = joint_positions

class MultiRobotSimulation:
    """Main simulation class for managing multiple robots"""
    
    def __init__(self, num_robots=100, use_gui=True):
        self.num_robots = num_robots
        self.robots = []
        self.robot_bodies = []
        self.use_gui = use_gui
        
        # Initialize PyBullet
        if use_gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
            
        # Set search path for URDF files
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Configure simulation parameters
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1./240.)  # Default timestep
        p.setRealTimeSimulation(0)  # Disable real-time for speed
        
        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        
    def create_simple_robot(self, position):
        """Create a simple robot using basic shapes"""
        # Create robot body (box)
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.3, 0.2])
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.3, 0.2], 
                                         rgbaColor=[random.random(), random.random(), random.random(), 1.0])
        
        robot_body = p.createMultiBody(baseMass=1.0,
                                     baseCollisionShapeIndex=collision_shape,
                                     baseVisualShapeIndex=visual_shape,
                                     basePosition=position)
        
        return robot_body
        
    def spawn_robots(self):
        """Spawn specified number of robots in random positions"""
        print(f"Spawning {self.num_robots} robots...")
        
        for i in range(self.num_robots):
            # Random position within a large area
            x = random.uniform(-20, 20)
            y = random.uniform(-20, 20)
            z = 1.0  # Above ground
            position = [x, y, z]
            
            # Create robot body
            robot_body = self.create_simple_robot(position)
            
            # Create robot object
            robot = Robot(i, position)
            
            self.robots.append(robot)
            self.robot_bodies.append(robot_body)
            
        print(f"Successfully spawned {len(self.robots)} robots")
        
    def apply_random_movements(self):
        """Apply random movement commands to all robots"""
        for i, robot in enumerate(self.robots):
            # Generate random linear velocity
            linear_vel = [random.uniform(-2, 2), random.uniform(-2, 2), 0]
            angular_vel = random.uniform(-1, 1)
            
            # Apply velocity to robot body
            p.resetBaseVelocity(self.robot_bodies[i], linear_vel, [0, 0, angular_vel])
            
    def check_collisions(self):
        """Check for collisions between robots"""
        collision_pairs = []
        
        for i in range(len(self.robot_bodies)):
            for j in range(i + 1, len(self.robot_bodies)):
                contact_points = p.getContactPoints(self.robot_bodies[i], self.robot_bodies[j])
                if contact_points:
                    collision_pairs.append((i, j))
                    
        return collision_pairs
        
    def run_simulation(self, duration=60, speed_multiplier=10):
        """Run the simulation for specified duration at specified speed"""
        print(f"Running simulation for {duration} seconds at {speed_multiplier}x speed...")
        
        # Spawn robots
        self.spawn_robots()
        
        # Simulation loop
        start_time = time.time()
        step_count = 0
        
        while time.time() - start_time < duration:
            # Apply random movements every 60 steps (0.25 seconds)
            if step_count % 60 == 0:
                self.apply_random_movements()
                
            # Check for collisions every 10 steps
            if step_count % 10 == 0:
                collisions = self.check_collisions()
                if collisions and len(collisions) > 0:
                    print(f"Collisions detected: {len(collisions)} pairs")
                    
            # Step simulation multiple times for speed increase
            for _ in range(speed_multiplier):
                p.stepSimulation()
                
            step_count += 1
            
            # Small delay to prevent overwhelming the system
            if self.use_gui:
                time.sleep(1.0 / 240.0 / speed_multiplier)
                
        print("Simulation completed!")
        
    def disconnect(self):
        """Clean up and disconnect from PyBullet"""
        p.disconnect()

def main():
    """Main function to run the multi-robot simulation"""
    print("Multi-Robot Simulation Starting...")
    print("Configuration:")
    print("- Number of robots: 100")
    print("- Speed multiplier: 10x")
    print("- Physics: Disabled for performance")
    print("- Collision detection: Enabled")
    
    # Create and run simulation
    sim = MultiRobotSimulation(num_robots=100, use_gui=True)
    
    try:
        sim.run_simulation(duration=30, speed_multiplier=10)
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    finally:
        sim.disconnect()
        print("Simulation ended")

if __name__ == "__main__":
    main()