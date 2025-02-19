from typing import Dict, Any, Set, Optional
from enum import Enum, auto
import numpy as np


from .robot import Robot
import pybullet as p

class VisualisationType(Enum):
    JOINT_AXES = auto()
    CENTER_OF_MASS = auto()
    CONTACT_POINTS = auto()


class Renderer:
    """ """
    def __init__(
            self,
            robot: Robot,
            physics_client: int,
            update_frequency: int = 240
    ):
        self.physics_client: int = physics_client
        self.robot = robot
        self.line_ids: Dict[str, int] = {} # gravity lines
        self.contact_points_ids: Dict[str, int] = {}
        self.rendered_items: Set[VisualisationType] = set()

        self.step_counter: int = 0
        self.update_frequency =  update_frequency# update every 100 step
        self.prev_com_position: Optional[np.ndarray] = None
        self.max_force_observed: float = 0.0
        self.reset()

    def init_visualization(self) -> None:

        POINT_HEIGHT = 1
        self.line_ids["com_point"] = p.addUserDebugLine(
            [0, 0, 0],
            [0, 0, POINT_HEIGHT],
            [1, 0, 0], # red
            lineWidth=3,
            physicsClientId=self.physics_client
        )

        # gravity vector
        self.line_ids["gravity_line"] = p.addUserDebugLine(
            [0, 0, 0.32],
            [0, 0, 0.0],
            [1, 0, 0], # blue
            lineWidth=5,
            physicsClientId=self.physics_client
        )

        self.line_ids["com_trail"] = p.addUserDebugLine(
            [0, 0, 0],
            [0, 0, 0],
            [1, 0, 0], # red
            lineWidth=3,
            physicsClientId=self.physics_client
        )



    def reset(self) -> None:
        for line_id in self.line_ids.values():
            p.removeUserDebugItem(line_id, physicsClientId=self.physics_client)
        self.line_ids.clear()
        self.contact_points_ids.clear()
        self.init_visualization()
        self.prev_com_position = self.robot.get_com()



    def update_com(self, position: np.ndarray )->None:
        """Visualize center of mass of robot"""
        position = np.array(position)

        # grab id of the com line and gravity line
        p.addUserDebugLine(
            position,
            [position[0], position[1], position[2] + 0.02],
            [0,0,0],
            lineWidth=3,
            replaceItemUniqueId=self.line_ids["com_point"],
            physicsClientId=self.physics_client
        )
        p.addUserDebugLine(
            position,
            [position[0], position[1], position[2] - 0.24],
            [0, 0, 0],
            lineWidth=3,
            replaceItemUniqueId=self.line_ids["gravity_line"],
            physicsClientId=self.physics_client
        )
        p.addUserDebugLine(
            self.prev_com_position,
            position,
            [1, 0, 0],
            lineWidth=3,
            replaceItemUniqueId=self.line_ids["com_trail"],
            physicsClientId=self.physics_client
        )
        self.prev_com_position = position

    def update_contacts(self) -> None:
        """Update visualization of contact points and forces"""
        # Remove old contact visualizations
        for line_id in self.contact_points_ids.values():
            p.removeUserDebugItem(line_id, physicsClientId=self.physics_client)
        self.contact_points_ids.clear()

        # Get all contact points involving the robot
        contact_points = p.getContactPoints(
            bodyA=self.robot.id,
            physicsClientId=self.physics_client
        )

        # Update maximum observed force for normalization
        for contact in contact_points:
            normal_force = contact[9]  # Index 9 contains normal force
            self.max_force_observed = max(self.max_force_observed, normal_force)

        # Visualize each contact point
        for i, contact in enumerate(contact_points):
            pos_on_robot = contact[5]  # Contact position on robot
            normal = contact[7]  # Contact normal direction
            normal_force = contact[9]  # Normal force magnitude

            # Calculate normal force line length (normalized)
            force_scale = 0.1  # Scale factor for visualization
            normalized_force = normal_force / self.max_force_observed
            line_length = normalized_force * force_scale

            # Calculate end point of normal force line
            end_point = [
                pos_on_robot[0] + normal[0] * line_length,
                pos_on_robot[1] + normal[1] * line_length,
                pos_on_robot[2] + normal[2] * line_length
            ]

            # Color based on force magnitude (red for high force, green for low)
            color = [normalized_force, 1.0 - normalized_force, 0]

            # Create or update line visualization
            # line_id = p.addUserDebugLine(
            #     pos_on_robot,
            #     end_point,
            #     color,
            #     lineWidth=2,
            #     physicsClientId=self.physics_client
            # )

            # self.contact_points_ids[f"contact_{i}"] = line_id

        # Remove old visualization if it exists
    def update(self) -> None:
        """Update all active visualizations"""
        self.step_counter += 1
        if self.step_counter % self.update_frequency != 0:
            return
        #
        self.update_com(self.robot.get_com())
        # self.robot.get_contact_points()
        # self.update_contacts()