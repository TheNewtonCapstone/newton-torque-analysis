import time
import numpy as np
from typing import List, Dict
import pybullet as p


class TrottingController:
    def __init__(self, robot, time_step=1/240):
        """ Initialize Trotting Controller with parameters for forward motion """
        self.robot = robot
        self.time_step = time_step
        self.phase = 0
        self.frequency = 1.2  # Hz - reduced for better stability
        self.swing_height = 0.6  # Height of leg lift during swing
        self.step_length = 0.2  # Length of forward/backward motion (increased)

        # Get the standing pose from the robot
        self.stance_pose = self.robot.get_standing_pose()

        # Forces for different phases
        self.stance_force = 15.0  # Higher force for pushing against ground
        self.swing_force = 5.0   # Lower force for leg movement

        # Debug counters
        self.steps = 0
        self.forward_speed = 0.1  # Target forward velocity (m/s)

    def step(self) -> Dict[str, List[float]]:
        """
        Generate the next step in the trotting gait
        Returns: Dictionary with leg positions
        """
        # Update phase
        self.phase = (self.phase + self.time_step * self.frequency) % 1.0

        # Create a fresh copy of the stance pose
        pose = {
            "fl": self.stance_pose["fl"].copy(),
            "fr": self.stance_pose["fr"].copy(),
            "hl": self.stance_pose["hl"].copy(),
            "hr": self.stance_pose["hr"].copy()
        }

        # Determine which diagonal pair is in swing phase
        if self.phase < 0.5:
            swing_legs = ["fl", "hr"]
            stance_legs = ["fr", "hl"]
        else:
            swing_legs = ["fr", "hl"]
            stance_legs = ["fl", "hr"]

        # Calculate normalized phase for swing legs
        if self.phase < 0.5:
            swing_phase = self.phase / 0.5
        else:
            swing_phase = (self.phase - 0.5) / 0.5

        # Apply leg motions for STANCE legs - these provide the propulsion
        for leg in stance_legs:
            # In stance phase, legs should push backward to propel the robot forward
            # For front legs: negative hip adjustment pushes backward
            # For hind legs: positive hip adjustment pushes backward
            if leg.startswith("f"):  # Front legs
                direction = -1.0  # Push backward
            else:  # Hind legs
                direction = 1.0   # Push backward

            # Create a backward-pushing motion during stance
            # Start with leg forward, then push back during stance
            stance_phase = swing_phase  # Same timing but different legs
            hip_stance_adjust = direction * self.step_length * (2 * stance_phase - 1)

            # Apply the stance motion (push backward)
            pose[leg][1] = self.stance_pose[leg][1] + hip_stance_adjust

            # Apply slight downward pressure during stance
            pose[leg][2] = self.stance_pose[leg][2] - 0.1  # Push down harder

            # Set higher forces for stance legs through joints
            if leg == "fl":
                for joint, angle in zip(self.robot.FL_joints, pose[leg]):
                    p.setJointMotorControl2(
                        self.robot.id, joint,
                        p.POSITION_CONTROL,
                        targetPosition=angle,
                        force=self.stance_force
                    )
            elif leg == "fr":
                for joint, angle in zip(self.robot.FR_joints, pose[leg]):
                    p.setJointMotorControl2(
                        self.robot.id, joint,
                        p.POSITION_CONTROL,
                        targetPosition=angle,
                        force=self.stance_force
                    )
            elif leg == "hl":
                for joint, angle in zip(self.robot.HL_joints, pose[leg]):
                    p.setJointMotorControl2(
                        self.robot.id, joint,
                        p.POSITION_CONTROL,
                        targetPosition=angle,
                        force=self.stance_force
                    )
            elif leg == "hr":
                for joint, angle in zip(self.robot.HR_joints, pose[leg]):
                    p.setJointMotorControl2(
                        self.robot.id, joint,
                        p.POSITION_CONTROL,
                        targetPosition=angle,
                        force=self.stance_force
                    )

        # Apply leg motions for SWING legs
        for leg in swing_legs:
            # For the height (knee) motion, use a parabolic trajectory
            height_factor = 4 * swing_phase * (1 - swing_phase)  # Parabolic curve: 0->1->0
            lift_amount = self.swing_height * height_factor

            # For front/back (hip) motion, we want a different trajectory:
            # Start backward, move forward during swing, end forward
            if leg.startswith("f"):  # Front legs
                direction = -1.0  # Moving from back to front
            else:  # Hind legs
                direction = 1.0   # Moving from front to back

            # Create a smooth forward motion during swing
            # Linear motion from back to front during swing
            forward_factor = swing_phase
            hip_swing_adjust = direction * self.step_length * (1 - 2 * forward_factor)

            # Apply the swing trajectory
            pose[leg][1] = self.stance_pose[leg][1] + hip_swing_adjust
            pose[leg][2] = self.stance_pose[leg][2] + lift_amount

            # Set lower forces for swing legs through joints
            if leg == "fl":
                for joint, angle in zip(self.robot.FL_joints, pose[leg]):
                    p.setJointMotorControl2(
                        self.robot.id, joint,
                        p.POSITION_CONTROL,
                        targetPosition=angle,
                        force=self.swing_force
                    )
            elif leg == "fr":
                for joint, angle in zip(self.robot.FR_joints, pose[leg]):
                    p.setJointMotorControl2(
                        self.robot.id, joint,
                        p.POSITION_CONTROL,
                        targetPosition=angle,
                        force=self.swing_force
                    )
            elif leg == "hl":
                for joint, angle in zip(self.robot.HL_joints, pose[leg]):
                    p.setJointMotorControl2(
                        self.robot.id, joint,
                        p.POSITION_CONTROL,
                        targetPosition=angle,
                        force=self.swing_force
                    )
            elif leg == "hr":
                for joint, angle in zip(self.robot.HR_joints, pose[leg]):
                    p.setJointMotorControl2(
                        self.robot.id, joint,
                        p.POSITION_CONTROL,
                        targetPosition=angle,
                        force=self.swing_force
                    )

        # Increment debug counter and print info periodically
        self.steps += 1
        if self.steps % 50 == 0:
            print(f"Phase: {self.phase:.2f}")
            print(f"Swing Legs: {swing_legs}")
            print(f"Stance Legs: {stance_legs}")
            print(f"Normalized Swing Phase: {swing_phase:.2f}")

            # Get base position and velocity for monitoring
            pos, orn = p.getBasePositionAndOrientation(self.robot.id)
            linear_vel, angular_vel = p.getBaseVelocity(self.robot.id)
            print(f"Position: {pos}, Forward velocity: {linear_vel[0]:.4f} m/s")

        return pose