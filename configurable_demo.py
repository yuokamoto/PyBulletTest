#!/usr/bin/env python3
"""
Configurable Multi-Robot Simulation Demo
Parameters: number of robots, realtime factor, duration
"""

import pybullet as p
import pybullet_data
import numpy as np
import random
import time
import argparse
import sys
import os

class ConfigurableRobotSimulation:
    """Configurable multi-robot simulation with adjustable parameters"""
    
    def __init__(self, num_robots=10, target_realtime_factor=1.0, time_step=1./240., use_gui=True, enable_physics=False, verbose=False):
        self.num_robots = num_robots
        self.target_realtime_factor = target_realtime_factor
        self.time_step = time_step
        self.use_gui = use_gui
        self.enable_physics = enable_physics
        self.verbose = verbose
        self.robots = []
        self.robot_bodies = []
        self.robot_types = []  # Track which type each robot is
        self.robot_joints = []  # Track joint info for each robot
        self.robot_constraints = []  # Track constraints for mobile robots
        self.collision_count = 0
        self.step_count = 0
        self.start_time = None
        
        # Display text IDs for GUI
        self.text_ids = []
        self.last_display_update = 0
        self.last_movement_update = 0
        self.last_collision_check = 0
        
        # Initialize PyBullet
        if use_gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
            
        # Set search path for URDF files
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Configure simulation parameters
        if self.enable_physics:
            p.setGravity(0, 0, -9.81)
        else:
            # Disable physics by setting gravity to zero and disabling constraints
            p.setGravity(0, 0, 0)
            
        p.setTimeStep(self.time_step)
        p.setRealTimeSimulation(0)  # Always disable for controlled speed
        
        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        
        if self.verbose:
            print(f"Simulation initialized with {num_robots} robots at {target_realtime_factor}x target speed")
            print(f"Time step: {self.time_step:.6f}s ({1/self.time_step:.1f} Hz)")
            print(f"Physics: {'Enabled' if self.enable_physics else 'Disabled (performance mode)'}")
        
    def create_robot(self, position, robot_id):
        """Create a robot (either arm_robot or mobile_robot) with random selection"""
        # Randomly choose robot type
        robot_types = ['arm_robot', 'mobile_robot']
        robot_type = random.choice(robot_types)
        
        # Load appropriate URDF
        urdf_path = os.path.join('robots', f'{robot_type}.urdf')
        
        # Set orientation and position based on robot type
        if robot_type == 'arm_robot':
            orientation = p.getQuaternionFromEuler([0, 0, 0])  # Upright
            # Place arm robots on the ground (z=0)
            arm_position = [position[0], position[1], 0]
        else:  # mobile_robot
            orientation = p.getQuaternionFromEuler([0, 0, 0])  # Normal orientation
            arm_position = position
        
        # Load robot from URDF with optimized settings
        if self.enable_physics:
            robot_body = p.loadURDF(urdf_path, arm_position, orientation, useFixedBase=(robot_type == 'arm_robot'))
        else:
            # In no-physics mode, load faster without collision checking
            robot_body = p.loadURDF(urdf_path, arm_position, orientation, 
                                   useFixedBase=(robot_type == 'arm_robot'),
                                   flags=p.URDF_USE_INERTIA_FROM_FILE)
        
        # Get joint information for this robot (cached for performance)
        joint_info = []
        num_joints = p.getNumJoints(robot_body)
        for i in range(num_joints):
            joint_info.append(p.getJointInfo(robot_body, i))
        
        return robot_body, robot_type, joint_info
    
    def _hsv_to_rgb(self, h, s, v):
        """Convert HSV to RGB color space"""
        import colorsys
        return list(colorsys.hsv_to_rgb(h, s, v))
        
    def spawn_robots(self):
        """Spawn robots in a grid pattern to avoid initial overlaps"""
        if self.verbose:
            print(f"Spawning {self.num_robots} robots...")
            spawn_start = time.time()
        
        # Calculate grid dimensions
        grid_size = int(np.ceil(np.sqrt(self.num_robots)))
        spacing = 2.0  # Distance between robots
        
        # Pre-generate robot types for better performance
        robot_types_list = ['arm_robot', 'mobile_robot']
        selected_types = [random.choice(robot_types_list) for _ in range(self.num_robots)]
        
        # Disable rendering during spawn for speed
        if self.use_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        
        for i in range(self.num_robots):
            # Grid position
            row = i // grid_size
            col = i % grid_size
            
            # Calculate world position
            x = (col - grid_size/2) * spacing
            y = (row - grid_size/2) * spacing
            # Set different heights for different robot types
            if selected_types[i] == 'mobile_robot':
                z = 0.3  # Mobile robots slightly above ground
            else:  # arm_robot
                z = 0.0  # Arm robots on ground
            position = [x, y, z]
            
            # Create robot with pre-selected type
            robot_body, robot_type, joint_info = self.create_robot_fast(position, i, selected_types[i])
            self.robot_bodies.append(robot_body)
            self.robot_types.append(robot_type)
            self.robot_joints.append(joint_info)
            
            # For mobile robots, we'll handle yaw-only rotation in apply_random_movements
            # instead of using constraints which can be problematic
            self.robot_constraints.append(None)
            
        # Re-enable rendering
        if self.use_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            
        if self.verbose:
            spawn_time = time.time() - spawn_start
            print(f"Successfully spawned {len(self.robot_bodies)} robots in {grid_size}x{grid_size} grid")
            print(f"Spawn time: {spawn_time:.3f}s ({len(self.robot_bodies)/spawn_time:.1f} robots/sec)")
    
    def create_robot_fast(self, position, robot_id, robot_type):
        """Optimized robot creation with pre-selected type"""
        # Load appropriate URDF
        urdf_path = os.path.join('robots', f'{robot_type}.urdf')
        
        # Set orientation and position based on robot type
        orientation = p.getQuaternionFromEuler([0, 0, 0])
        if robot_type == 'mobile_robot':
            final_position = [position[0], position[1], 0.3]  # Mobile robots at height 0.3
        else:  # arm_robot
            final_position = [position[0], position[1], 0]  # Arm robots on ground
        
        # Load robot with optimized flags
        flags = p.URDF_USE_INERTIA_FROM_FILE
        if not self.enable_physics:
            flags |= p.URDF_IGNORE_COLLISION_SHAPES
            
        robot_body = p.loadURDF(urdf_path, final_position, orientation, 
                               useFixedBase=(robot_type == 'arm_robot'),
                               flags=flags)
        
        # Note: Mobile robot constraints will be handled in spawn_robots method
        
        # Get joint information
        joint_info = []
        num_joints = p.getNumJoints(robot_body)
        for i in range(num_joints):
            joint_info.append(p.getJointInfo(robot_body, i))
        
        return robot_body, robot_type, joint_info
        
    def apply_random_movements(self):
        """Apply random movement commands to all robots"""
        for i, (robot_body, robot_type) in enumerate(zip(self.robot_bodies, self.robot_types)):
            if robot_type == 'mobile_robot':
                # Generate random forward/backward and yaw velocities
                max_linear_speed = 2.0  # m/s
                max_angular_speed = 1.5  # rad/s
                
                forward_vel = random.uniform(-max_linear_speed, max_linear_speed)  # Forward/backward
                yaw_vel = random.uniform(-max_angular_speed, max_angular_speed)  # Yaw rotation
                
                # Apply velocity control - only forward/backward and yaw
                linear_vel = [forward_vel, 0, 0]  # Only X-direction movement
                angular_vel = [0, 0, yaw_vel]  # Only Z-axis (yaw) rotation
                
                # Enforce position and orientation constraints
                pos, orn = p.getBasePositionAndOrientation(robot_body)
                euler = p.getEulerFromQuaternion(orn)
                # Keep only yaw (Z rotation), reset pitch and roll to 0
                corrected_orn = p.getQuaternionFromEuler([0, 0, euler[2]])
                # Ensure robot stays at correct height (z=0.3)
                p.resetBasePositionAndOrientation(robot_body, [pos[0], pos[1], 0.3], corrected_orn)
                
                # Apply the velocity commands directly to the robot body
                p.resetBaseVelocity(robot_body, linear_vel, angular_vel)
                        
            elif robot_type == 'arm_robot':
                # Apply joint commands for arm robots (fixed to floor)
                joint_info = self.robot_joints[i]
                for j, info in enumerate(joint_info):
                    joint_type = info[2]
                    if joint_type == p.JOINT_REVOLUTE:  # Only control revolute joints
                        # Random joint position target
                        joint_limits = info[8:10]  # lower, upper limits
                        if joint_limits[0] < joint_limits[1]:  # Valid limits
                            target_pos = random.uniform(joint_limits[0], joint_limits[1])
                            p.setJointMotorControl2(robot_body, j, p.POSITION_CONTROL, targetPosition=target_pos)
                        else:
                            # No limits or invalid limits, use small random movement
                            target_pos = random.uniform(-1.0, 1.0)
                            p.setJointMotorControl2(robot_body, j, p.POSITION_CONTROL, targetPosition=target_pos)
            
    def check_collisions(self):
        """Check for collisions and count them"""
        collision_pairs = []
        
        for i in range(len(self.robot_bodies)):
            for j in range(i + 1, len(self.robot_bodies)):
                contact_points = p.getContactPoints(self.robot_bodies[i], self.robot_bodies[j])
                if contact_points:
                    collision_pairs.append((i, j))
                    
        return collision_pairs
    
    def update_display_text(self, elapsed_time):
        """Update on-screen display text with simulation statistics"""
        if not self.use_gui:
            return
            
        # Calculate current statistics
        sim_time = self.step_count * self.time_step  # simulation time in seconds
        if elapsed_time > 0:
            actual_realtime_factor = sim_time / elapsed_time
        else:
            actual_realtime_factor = 0
            
        # Remove old text
        for text_id in self.text_ids:
            try:
                p.removeUserDebugItem(text_id)
            except:
                pass
        self.text_ids.clear()
        
        # Display information
        lines = [
            f"Simulation Time: {sim_time:.1f}s",
            f"Real Time: {elapsed_time:.1f}s",
            f"Target Speed: {self.target_realtime_factor:.1f}x",
            f"Actual Speed: {actual_realtime_factor:.1f}x",
            f"Time Step: {self.time_step:.4f}s ({1/self.time_step:.0f}Hz)",
            f"Physics: {'ON' if self.enable_physics else 'OFF'}",
            f"Robots: {self.num_robots} (Arms: {self.robot_types.count('arm_robot')}, Mobile: {self.robot_types.count('mobile_robot')})",
            f"Collisions: {self.collision_count}",
            f"Step: {self.step_count}"
        ]
        
        # Add text to display
        for i, line in enumerate(lines):
            text_id = p.addUserDebugText(
                text=line,
                textPosition=[0, 0, 8 + i * 0.5],  # Position in world coordinates
                textColorRGB=[1, 1, 1],  # White text
                textSize=1.5,
                parentObjectUniqueId=-1,
                parentLinkIndex=-1
            )
            self.text_ids.append(text_id)
    
    def clear_display_text(self):
        """Clear all on-screen text"""
        if not self.use_gui:
            return
            
        for text_id in self.text_ids:
            try:
                p.removeUserDebugItem(text_id)
            except:
                pass
        self.text_ids.clear()
    
    def print_statistics(self, elapsed_time):
        """Print simulation statistics"""
        if elapsed_time > 0:
            sim_speed = (self.step_count * self.time_step) / elapsed_time  # simulation time / real time
            steps_per_sec = self.step_count / elapsed_time
            
            print(f"\n=== Simulation Statistics ===")
            print(f"Robots: {self.num_robots} (Arms: {self.robot_types.count('arm_robot')}, Mobile: {self.robot_types.count('mobile_robot')})")
            print(f"Physics: {'Enabled' if self.enable_physics else 'Disabled (performance mode)'}")
            print(f"Time step: {self.time_step:.6f}s ({1/self.time_step:.1f} Hz)")
            print(f"Target realtime factor: {self.target_realtime_factor}x")
            print(f"Actual realtime factor: {sim_speed:.2f}x")
            print(f"Steps per second: {steps_per_sec:.1f}")
            print(f"Total collisions detected: {self.collision_count}")
            print(f"Simulation time: {self.step_count * self.time_step:.2f}s")
            print(f"Real time: {elapsed_time:.2f}s")
        
    def run_simulation(self, duration=30):
        """Run the simulation for specified duration"""
        if self.verbose:
            print(f"Running simulation for {duration} seconds at {self.target_realtime_factor}x speed...")
        
        # Spawn robots
        self.spawn_robots()
        
        # Simulation loop with precise timing control
        self.start_time = time.time()
        last_step_time = self.start_time
        
        try:
            while True:
                current_time = time.time()
                elapsed_time = current_time - self.start_time
                
                # Check if duration exceeded
                if elapsed_time >= duration:
                    break
                
                # Calculate target simulation time based on target realtime factor
                target_sim_time = elapsed_time * self.target_realtime_factor
                current_sim_time = self.step_count * self.time_step
                
                # Time-based updates (independent of time_step)
                # Apply random movements every 0.25 seconds
                if current_time - self.last_movement_update >= 0.25:
                    self.apply_random_movements()
                    self.last_movement_update = current_time
                    
                # Check for collisions every 0.1 seconds
                if current_time - self.last_collision_check >= 0.1:
                    collisions = self.check_collisions()
                    if collisions:
                        self.collision_count += len(collisions)
                        if self.verbose and len(collisions) > 0:
                            print(f"Time {elapsed_time:.1f}s: {len(collisions)} collision pairs detected")
                    self.last_collision_check = current_time
                
                # Update display text every 0.1 seconds for smooth updates
                if self.use_gui and current_time - self.last_display_update >= 0.1:
                    self.update_display_text(elapsed_time)
                    self.last_display_update = current_time
                
                # Only step simulation if we're behind the target
                if current_sim_time < target_sim_time:
                    # Calculate how many steps we need to catch up
                    steps_needed = int((target_sim_time - current_sim_time) / self.time_step)
                    steps_needed = max(1, min(steps_needed, 10))  # Limit to 1-10 steps per frame
                    
                    # If physics is disabled, we can process multiple steps efficiently
                    if not self.enable_physics:
                        steps_needed = min(steps_needed, 5)  # Slightly more conservative for no-physics
                    else:
                        steps_needed = 1  # Single step for physics mode
                    
                    # Step simulation
                    for _ in range(steps_needed):
                        p.stepSimulation()
                        self.step_count += 1
                        
                        # Update current simulation time
                        current_sim_time = self.step_count * self.time_step
                        if current_sim_time >= target_sim_time:
                            break
                
                # Adaptive frame rate control based on simulation performance
                if current_sim_time < target_sim_time:
                    # We're behind target - minimize sleep to catch up
                    if self.use_gui:
                        time.sleep(0.001)  # Minimal sleep for GUI responsiveness
                    else:
                        time.sleep(0.0001)  # Nearly no sleep for headless
                else:
                    # We're at or ahead of target - normal frame rate control
                    if self.use_gui:
                        time.sleep(1.0 / 60.0)  # 60 FPS for smooth display
                    else:
                        time.sleep(0.001)  # Minimal sleep to prevent CPU overuse
                    
        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        
        # Clear display text and print final statistics
        self.clear_display_text()
        final_time = time.time() - self.start_time
        self.print_statistics(final_time)
        
    def disconnect(self):
        """Clean up and disconnect from PyBullet"""
        self.clear_display_text()
        # Clean up constraints
        for constraint_id in getattr(self, 'robot_constraints', []):
            if constraint_id is not None:
                try:
                    p.removeConstraint(constraint_id)
                except:
                    pass
        p.disconnect()

def main():
    """Main function with command line argument parsing"""
    parser = argparse.ArgumentParser(
        description="Configurable Multi-Robot Simulation Demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                              # Default: 10 robots, 1x speed, physics ON
  %(prog)s --robots 50 --speed 5        # 50 robots at 5x speed
  %(prog)s --robots 100 --speed 10 --duration 60  # 100 robots, 10x speed, 60s
  %(prog)s --robots 200 --no-physics --no-gui  # Max performance: no physics, no GUI
  %(prog)s --robots 5 --speed 0.5 --timestep 0.01  # 5 robots, 0.5x speed, 100Hz
  %(prog)s --no-physics --speed 20 --verbose  # No physics, 20x speed with verbose
        """
    )
    
    parser.add_argument('--robots', '-r', type=int, default=10,
                      help='Number of robots to simulate (default: 10)')
    parser.add_argument('--speed', '-s', type=float, default=1.0,
                      help='Target realtime factor - simulation speed multiplier (default: 1.0)')
    parser.add_argument('--timestep', '-t', type=float, default=1./240.,
                      help='Simulation time step in seconds (default: 0.004167, 240Hz)')
    parser.add_argument('--duration', '-d', type=int, default=30,
                      help='Simulation duration in seconds (default: 30)')
    parser.add_argument('--gui', action='store_true', default=True,
                      help='Enable GUI (default: enabled)')
    parser.add_argument('--no-gui', action='store_true',
                      help='Disable GUI for better performance')
    parser.add_argument('--physics', action='store_true',
                      help='Enable physics simulation (disabled by default for performance)')
    parser.add_argument('--no-physics', action='store_true',
                      help='Explicitly disable physics simulation (default behavior)')
    parser.add_argument('--verbose', '-v', action='store_true',
                      help='Enable verbose output')
    
    args = parser.parse_args()
    
    # Validate arguments
    if args.robots < 1:
        print("Error: Number of robots must be at least 1")
        sys.exit(1)
    
    if args.speed <= 0:
        print("Error: Speed multiplier must be positive")
        sys.exit(1)
        
    if args.timestep <= 0 or args.timestep > 0.1:
        print("Error: Time step must be positive and reasonable (0 < timestep <= 0.1)")
        sys.exit(1)
        
    if args.duration < 1:
        print("Error: Duration must be at least 1 second")
        sys.exit(1)
    
    # Handle GUI and physics flags
    use_gui = args.gui and not args.no_gui
    # Physics is disabled by default, enabled only if --physics flag is used
    enable_physics = args.physics and not args.no_physics
    
    # Display configuration
    print("=== Multi-Robot Simulation Demo ===")
    print(f"Configuration:")
    print(f"  Robots: {args.robots}")
    print(f"  Target Realtime factor: {args.speed}x")
    print(f"  Time step: {args.timestep:.6f}s ({1/args.timestep:.1f} Hz)")
    print(f"  Duration: {args.duration} seconds")
    print(f"  Physics: {'Enabled' if enable_physics else 'Disabled (performance mode)'}")
    print(f"  GUI: {'Enabled' if use_gui else 'Disabled'}")
    print(f"  Verbose: {'Enabled' if args.verbose else 'Disabled'}")
    print()
    
    # Robot behavior info and performance warnings
    if args.robots > 100:
        print("⚠️  Warning: Large number of robots may impact performance")
    if args.speed > 10 and use_gui:
        print("⚠️  Warning: High speed with GUI may cause visual lag")
    if args.robots > 50 and args.speed > 5:
        print("⚠️  Warning: High robot count + speed may stress system")
    
    print("Robot behaviors:")
    print("  - Arm robots: Fixed to ground (z=0), joint position control")
    print("  - Mobile robots: Height z=0.3, direct velocity control (forward/back + yaw only)")
    
    print("Starting simulation... (Press Ctrl+C to stop early)")
    print()
    
    # Create and run simulation
    sim = ConfigurableRobotSimulation(
        num_robots=args.robots,
        target_realtime_factor=args.speed,
        time_step=args.timestep,
        use_gui=use_gui,
        enable_physics=enable_physics,
        verbose=args.verbose
    )
    
    try:
        sim.run_simulation(duration=args.duration)
    finally:
        sim.disconnect()
        print("Simulation ended")

if __name__ == "__main__":
    main()