import time
import numpy as np
from typing import List, Dict
import pybullet as p


class WalkingController:
    def __init__(self, robot, time_step=1 / 240):
        """ Initialize Walking Controller with parameters for forward motion """
        self.robot = robot
        self.time_step = time_step
        self.phase = 0
        self.frequency = 1  # Hz - slower than trotting for stability
        self.swing_height = 0.5  # Height of leg lift during swing
        self.step_length = 0.25  # Length of forward/backward motion

        # Get the standing pose from the robot
        self.stance_pose = self.robot.get_standing_pose()

        # Forces for different phases - set to 3 Nm for each joint
        self.stance_force = 3.0  # Force for stance phase (pushing against ground)
        self.swing_force = 3.0  # Force for swing phase (leg movement)

        # Debug counters
        self.steps = 0
        self.forward_speed = 0.08  # Target forward velocity (m/s)

        # Define the leg sequence for walking gait (4-beat)
        self.leg_sequence = ["fl", "fr", "hr", "hl"]  # Standard walking sequence
        self.num_legs = len(self.leg_sequence)

    def step(self) -> Dict[str, List[float]]:
        """
        Generate the next step in the walking gait
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

        # For walking gait, determine which leg is in swing phase
        # Each leg gets 1/4 of the cycle, with 25% duty factor
        leg_phase_duration = 1.0 / self.num_legs
        swing_leg_idx = int(self.phase / leg_phase_duration)
        swing_leg = self.leg_sequence[swing_leg_idx]

        # Calculate normalized phase for the current swing leg
        swing_phase = (self.phase - swing_leg_idx * leg_phase_duration) / leg_phase_duration

        # All legs except the swing leg are in stance phase
        stance_legs = [leg for leg in self.leg_sequence if leg != swing_leg]

        # Apply leg motions for STANCE legs - provide propulsion and stability
        for leg in stance_legs:
            # In stance phase, legs should push backward to propel the robot forward
            # For front legs: negative hip adjustment pushes backward
            # For hind legs: positive hip adjustment pushes backward
            if leg.startswith("f"):  # Front legs
                direction = -1.0  # Push backward
            else:  # Hind legs
                direction = 1.0  # Push backward

            # Create a backward-pushing motion during stance
            # For walking, we want a smoother, more continuous push
            stance_phase = 0.5  # Middle of stance
            hip_stance_adjust = direction * self.step_length * 0.5  # Consistent push

            # Apply the stance motion (push backward)
            pose[leg][1] = self.stance_pose[leg][1] + hip_stance_adjust

            # Apply slight downward pressure during stance for better grip
            pose[leg][2] = self.stance_pose[leg][2] - 0.05

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

        # Apply leg motion for the SWING leg
        # For the height (knee) motion, use a parabolic trajectory
        height_factor = 4 * swing_phase * (1 - swing_phase)  # Parabolic curve: 0->1->0
        lift_amount = self.swing_height * height_factor

        # For front/back (hip) motion, we want a smooth trajectory:
        # Start backward, move forward during swing, end forward
        if swing_leg.startswith("f"):  # Front legs
            direction = -1.0  # Moving from back to front
        else:  # Hind legs
            direction = 1.0  # Moving from front to back

        # Create a smooth forward motion during swing
        # From back to front during swing (linear interpolation)
        forward_factor = swing_phase
        hip_swing_adjust = direction * self.step_length * (1 - 2 * forward_factor)

        # Apply the swing trajectory to the current swing leg
        pose[swing_leg][1] = self.stance_pose[swing_leg][1] + hip_swing_adjust
        pose[swing_leg][2] = self.stance_pose[swing_leg][2] + lift_amount

        # Set lower forces for swing leg through joints
        if swing_leg == "fl":
            for joint, angle in zip(self.robot.FL_joints, pose[swing_leg]):
                p.setJointMotorControl2(
                    self.robot.id, joint,
                    p.POSITION_CONTROL,
                    targetPosition=angle,
                    force=self.swing_force
                )
        elif swing_leg == "fr":
            for joint, angle in zip(self.robot.FR_joints, pose[swing_leg]):
                p.setJointMotorControl2(
                    self.robot.id, joint,
                    p.POSITION_CONTROL,
                    targetPosition=angle,
                    force=self.swing_force
                )
        elif swing_leg == "hl":
            for joint, angle in zip(self.robot.HL_joints, pose[swing_leg]):
                p.setJointMotorControl2(
                    self.robot.id, joint,
                    p.POSITION_CONTROL,
                    targetPosition=angle,
                    force=self.swing_force
                )
        elif swing_leg == "hr":
            for joint, angle in zip(self.robot.HR_joints, pose[swing_leg]):
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
            print(f"Swing Leg: {swing_leg}")
            print(f"Stance Legs: {stance_legs}")
            print(f"Normalized Swing Phase: {swing_phase:.2f}")

            # Get base position and velocity for monitoring
            pos, orn = p.getBasePositionAndOrientation(self.robot.id)
            linear_vel, angular_vel = p.getBaseVelocity(self.robot.id)
            print(f"Position: {pos}, Forward velocity: {linear_vel[0]:.4f} m/s")

        return pose