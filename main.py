# Import required libraries
import pybullet as p
import numpy as np
import time
import pybullet_data


def setup_simulation():
    physClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    plane = p.loadURDF("plane.urdf")

    robot = p.loadURDF("newton/newton.urdf", [0, 0, 0.33])

    return physClient, robot


def add_gravity_vector_visualization(start_pos=[1, 0, 0.33], end_pos=[1, 0, 0.23]):
    p.addUserDebugLine(start_pos, end_pos, [1, 0, 0], 2)  # Red line


def get_base_pose(robot):
    pos, orn = p.getBasePositionAndOrientation(robot)
    euler = p.getEulerFromQuaternion(orn)
    return pos, euler


def print_com_data(robot):
    num_joints = p.getNumJoints(robot)
    total_mass = 0
    weighted_pos = np.zeros(3)

    dyn_info = p.getDynamicsInfo(robot, -1)  # -1 for base link
    base_mass = dyn_info[0]
    base_pos, base_orn = p.getBasePositionAndOrientation(robot)
    total_mass += base_mass
    weighted_pos += np.array(base_pos) * base_mass

    print(f"\nBase mass: {base_mass:.3f} kg")
    print(f"Base position: {base_pos}")

    for i in range(num_joints):
        dyn_info = p.getDynamicsInfo(robot, i)
        link_mass = dyn_info[0]
        local_inertial_pos = dyn_info[3]

        link_state = p.getLinkState(robot, i)
        link_com_pos = link_state[0]  # World position of center of mass

        total_mass += link_mass
        weighted_pos += np.array(link_com_pos) * link_mass

        print(f"\nLink {i} ({p.getJointInfo(robot, i)[1].decode('utf-8')}):")
        print(f"Mass: {link_mass:.3f} kg")
        print(f"CoM Position: {link_com_pos}")

    # Calculate overall center of mass
    if total_mass > 0:
        com = weighted_pos / total_mass
        print(f"\nTotal mass: {total_mass:.3f} kg")
        print(f"Overall Center of Mass: {com}")


def set_standing_pose(robot):
    # note angles are in radians
    standing_pose = {
        "FL": [0.0, 0.5, -1.0], 
        "FR": [0.0, 0.5, -1.0],
        "HL": [0.0, 0.5, -1.0],
        "HR": [0.0, 0.5, -1.0]
    }

    FL_joints = [0, 1, 2]  
    FR_joints = [4, 5, 6]  
    HL_joints = [8, 9, 10]  
    HR_joints = [12, 13, 14]  

    # Set joint positions for each leg
    for joints, angles in zip([FL_joints, FR_joints, HL_joints, HR_joints],
                              [standing_pose["FL"], standing_pose["FR"],
                               standing_pose["HL"], standing_pose["HR"]]):
        for joint, angle in zip(joints, angles):
            p.resetJointState(robot, joint, angle)
            p.setJointMotorControl2(robot, joint,
                                    p.POSITION_CONTROL,
                                    targetPosition=angle,
                                    force=1000)


def monitor_robot_state(robot, duration=10.0, dt=0.1):
    # Enable torque sensors for all joints
    for i in range(p.getNumJoints(robot)):
        p.enableJointForceTorqueSensor(robot, i, enableSensor=1)

    start_time = time.time()
    while time.time() - start_time < duration:
        pos, euler = get_base_pose(robot)
        print("\n=== Robot State ===")
        print(f"Base Position: {[f'{x:.3f}' for x in pos]}")
        print(f"Base Orientation (deg): {[f'{np.degrees(x):.2f}' for x in euler]}")

        # Print joint torques
        print("\n=== Joint Torques Analysis ===")
        print("\nLeg      | HAA (N·m)  | HFE (N·m)  | KFE (N·m) ")
        print("---------------------------------------------")

        legs = {
            "FL": [0, 1, 2],  # Front Left
            "FR": [4, 5, 6],  # Front Right
            "HL": [8, 9, 10],  # Hind Left
            "HR": [12, 13, 14]  # Hind Right
        }

        for leg_name, joints in legs.items():
            torques = []
            for idx in joints:
                joint_state = p.getJointState(robot, idx)
                torques.append(joint_state[3])  # index 3 is applied torque

            print(f"{leg_name:<8} | {torques[0]:>9.3f} | {torques[1]:>9.3f} | {torques[2]:>9.3f}")

        p.stepSimulation()
        time.sleep(dt)


def main():
    # Setup simulation
    physClient, robot = setup_simulation()

    add_gravity_vector_visualization()

    print("\n============================================")
    print_com_data(robot)

    set_standing_pose(robot)

    # Let simulation settle
    for _ in range(1000):
    # while True:
        p.stepSimulation()

        # Monitor robot state
        monitor_robot_state(robot)

    p.disconnect()


if __name__ == "__main__":
    main()