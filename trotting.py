import pybullet as p
import numpy as np
import time
import pybullet_data
import math


def setup_simulation():
    """Initialize PyBullet simulation environment"""
    pass

def define_robot_config():
    """Define the robot configuration parameters"""
    pass
    # Robot dimensions and parameters


def leg_inverse_kinematics(x, y, z, leg_type, config):
    """
    Calculate joint angles for desired foot position

    Args:
        x, y, z: Desired foot position in leg frame
        leg_type: 'FL', 'FR', 'HL', or 'HR' to handle different leg orientations
        config: Robot configuration

    Returns:
        List of joint angles [HAA, HFE, KFE]
    """
    # Get leg dimensions
    L1 = config["upper_leg_length"]  # Upper leg length
    L2 = config["lower_leg_length"]  # Lower leg length

    # HAA calculation (abduction/adduction)
    haa = np.arctan2(y, x)

    # Calculate leg plane distance (horizontal distance from hip to foot)
    r = np.sqrt(x ** 2 + y ** 2)

    # Ensure the point is reachable
    D = (r ** 2 + z ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    D = np.clip(D, -1.0, 1.0)  # Clamp to handle numerical issues

    # HFE and KFE calculation using cosine law
    kfe = np.arccos(D)
    alpha = np.arctan2(z, r)
    beta = np.arctan2(L2 * np.sin(kfe), L1 + L2 * np.cos(kfe))
    hfe = alpha - beta

    # Adjust for leg orientation conventions (if needed)
    if leg_type in ['FR', 'HR']:
        haa = -haa  # Mirror for right legs

    # Adjust angles based on the robot's joint convention
    kfe = -kfe  # Flip KFE direction (based on the standing pose in the provided code)

    return [haa, hfe, kfe]


def generate_foot_trajectory(phase, leg_type, config):
    """
    Generate foot trajectory for a given phase and leg

    Args:
        phase: Current phase in the gait cycle [0, 2π]
        leg_type: 'FL', 'FR', 'HL', or 'HR'
        config: Robot configuration dictionary

    Returns:
        Foot position (x, y, z) in leg frame
    """
    # Get parameters
    step_length = config["step_length"]
    step_height = config["step_height"]

    # Calculate base foot position for this leg (in hip frame)
    if leg_type in ['FL', 'FR']:  # Front legs
        x0 = config["body_length"] / 4  # Start a bit forward
    else:  # Hind legs
        x0 = -config["body_length"] / 4  # Start a bit backward

    if leg_type in ['FL', 'HL']:  # Left legs
        y0 = config["body_width"] / 2  # Outward
    else:  # Right legs
        y0 = -config["body_width"] / 2  # Outward

    z0 = -config["leg_length"] * 0.8  # Default height (80% of full extension)

    # Apply leg phase offset
    adjusted_phase = (phase + config["phase_offsets"][leg_type]) % (2 * np.pi)

    # Divide gait into stance (0 to π) and swing (π to 2π) phases
    if adjusted_phase < np.pi:  # Stance phase - foot on ground
        # Linear movement from front to back
        stance_phase = adjusted_phase / np.pi
        x = x0 + step_length * (0.5 - stance_phase)
        y = y0
        z = z0
    else:  # Swing phase - foot in air
        # Semi-circular trajectory from back to front
        swing_phase = (adjusted_phase - np.pi) / np.pi
        x = x0 + step_length * (swing_phase - 0.5)
        y = y0
        z = z0 + step_height * np.sin(swing_phase * np.pi)

    return x, y, z


def update_config_from_gui():
    """Read parameters from the GUI sliders"""
    config_updates = {
        "step_height": p.readUserDebugParameter(0),
        "step_length": p.readUserDebugParameter(1),
        "gait_frequency": p.readUserDebugParameter(2)
    }
    return config_updates


def set_standing_pose(robot, config):
    """Set the robot to a standing pose"""
    # Use the standing angles from config
    for leg_name, joints in config["legs"].items():
        angles = config["standing_angles"][leg_name]
        for joint, angle in zip(joints, angles):
            p.resetJointState(robot, joint, angle)
            p.setJointMotorControl2(robot, joint,
                                    p.POSITION_CONTROL,
                                    targetPosition=angle,
                                    force=1000)


def add_debug_visualization(robot, config):
    """Add visual debugging elements for foot trajectories"""
    # Create debug visualization items for each foot
    debug_items = {}
    colors = {
        "FL": [1, 0, 0],  # Red
        "FR": [0, 1, 0],  # Green
        "HL": [0, 0, 1],  # Blue
        "HR": [1, 1, 0]  # Yellow
    }

    for leg_name in config["legs"]:
        # Debug sphere for foot position
        debug_items[leg_name] = p.addUserDebugParameter(f"{leg_name} Foot Debug", 0, 1, 0)

    return debug_items


def update_debug_visualization(robot, config, debug_items):
    """Update debug visualization for foot positions and trajectories"""
    # Get base position and orientation
    base_pos, base_orn = p.getBasePositionAndOrientation(robot)

    for leg_name, joints in config["legs"].items():
        # Get the foot position (end effector)
        foot_state = p.getLinkState(robot, joints[2])  # KFE joint's state
        foot_pos = foot_state[0]  # World position

        # Draw a small sphere at foot position
        p.addUserDebugLine(foot_pos,
                           [foot_pos[0], foot_pos[1], foot_pos[2] - 0.05],
                           [1, 0, 0],
                           lineWidth=2,
                           lifeTime=0.1)


def trotting_gait_controller(robot, config, current_time):
    """
    Main trotting gait controller

    Args:
        robot: PyBullet robot object
        config: Robot configuration dictionary
        current_time: Current simulation time
    """
    # Calculate current phase in gait cycle
    frequency = config["gait_frequency"]
    phase = (2 * np.pi * frequency * current_time) % (2 * np.pi)

    # For each leg, calculate foot position and joint angles
    for leg_name, joints in config["legs"].items():
        # Generate foot trajectory for this leg
        x, y, z = generate_foot_trajectory(phase, leg_name, config)

        # Calculate inverse kinematics
        target_angles = leg_inverse_kinematics(x, y, z, leg_name, config)

        # Apply joint commands
        for joint_idx, angle in zip(joints, target_angles):
            p.setJointMotorControl2(
                robot,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=angle,
                force=1000,
                maxVelocity=8.0  # Limit motor velocity for smooth motion
            )


def get_base_pose(robot):
    """Get the position and orientation of the robot base"""
    pos, orn = p.getBasePositionAndOrientation(robot)
    euler = p.getEulerFromQuaternion(orn)
    return pos, euler


def main():
    """Main function to run the simulation"""
    # Setup simulation
    physClient, robot = setup_simulation()

    # Define robot configuration
    config = define_robot_config()

    # Add debug visualization
    debug_items = add_debug_visualization(robot, config)

    # Set initial standing pose
    set_standing_pose(robot, config)

    # Let simulation settle
    print("Settling robot in standing pose...")
    for _ in range(100):
        p.stepSimulation()
        time.sleep(0.01)

    print("\n============================================")
    print("Starting trotting gait...")

    # Run simulation with trotting gait
    start_time = time.time()
    sim_duration = 30.0  # Run for 30 seconds
    dt = 1.0 / config["control_freq"]  # Control period (20ms for 50Hz)

    last_control_time = 0
    last_print_time = 0

    while time.time() - start_time < sim_duration:
        current_time = time.time() - start_time

        # Update configuration from GUI sliders
        gui_updates = update_config_from_gui()
        for key, value in gui_updates.items():
            config[key] = value

        # Run controller at specified frequency
        if current_time - last_control_time >= dt:
            # Apply trotting gait controller
            trotting_gait_controller(robot, config, current_time)
            last_control_time = current_time

            # Update debug visualization
            update_debug_visualization(robot, config, debug_items)

        # Step the simulation
        p.stepSimulation()

        # Print status periodically
        if current_time - last_print_time >= 1.0:  # Print every second
            pos, euler = get_base_pose(robot)
            print(f"\nTime: {current_time:.1f}s")
            print(f"Base Position: {[f'{x:.3f}' for x in pos]}")
            print(f"Base Orientation (deg): {[f'{math.degrees(x):.2f}' for x in euler]}")
            last_print_time = current_time

        # Sleep to maintain real-time factor
        time.sleep(0.001)  # Small sleep to prevent CPU hogging

    print("Simulation complete!")
    p.disconnect()


if __name__ == "__main__":
    main()