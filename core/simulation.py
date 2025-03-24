import time
from optparse import Option
import numpy as np
from typing import Tuple, List, Dict, Any, Optional
import pybullet as p
from anyio import sleep
from pprint import pprint

from .parameters import TimeSteppingParams, SolverParameters
from .robot import Robot, JointInfo
from .renderer import Renderer

class Simulation:
    def __init__(
            self,
            solver_iterations: int=SolverParameters.num_solver_iterations,
            time_step: float=TimeSteppingParams.fixed_timestep,
            gui_mode: int=p.GUI,
            controller: Optional[Any] = None
    ):
        self.solver_iterations = solver_iterations
        self.time_step = time_step
        self.gui_mode = gui_mode

        self.phys_client: Optional[int] = None

        # robot parameters
        self.robot: Optional[Robot] = None
        self.robot_id: Optional[int] = None
        self.ground_id: Optional[int] = None

        self.renderer: Optional [Renderer] = None
        self.controller: Optional[Any] = controller


    def connect(self):
        """ Establish connection to physics server"""
        self.phys_client = p.connect(self.gui_mode)
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        p.setPhysicsEngineParameter(numSolverIterations=self.solver_iterations)
        p.setTimeStep(self.time_step)


    def set_controller(self, controller: Any):
        self.controller = controller
    def load_ground_plane(self):
        import pybullet_data
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.ground_id = p.loadURDF("plane.urdf")
        p.changeDynamics(
            self.ground_id,
            -1,  # -1 represents the base link
            lateralFriction=1.0,
            spinningFriction=0.4,
            rollingFriction=0.15,
            frictionAnchor=1
        )


    def get_robot(self):
        return self.robot

    def run(self, duration:float=30.0):
        start_time = time.time()
        while time.time() - start_time < duration:
            self.step()
            time.sleep(1/240)

    def load_robot(self, urdf_path: str, start_pos: List[float]):
        # Load robot, not starting position is the base (X, Y, Z) position
        self.robot_id = p.loadURDF(urdf_path, start_pos)
        self.robot = Robot(self.robot_id, self.phys_client)
        self.renderer = Renderer(robot=self.robot, physics_client=self.phys_client)
        self.robot.set_standing_pose()

        def load_robot(self, urdf_path: str, start_pos: List[float]):
            # Load robot, not starting position is the base (X, Y, Z) position
            self.robot_id = p.loadURDF(urdf_path, start_pos)
            self.robot = Robot(self.robot_id, self.phys_client)
            self.renderer = Renderer(robot=self.robot, physics_client=self.phys_client)
            self.robot.set_standing_pose()
            for leg_name, ankle_idx in [
                ("FL", self.robot.legs["FL"].ankle_idx),
                ("FR", self.robot.legs["FR"].ankle_idx),
                ("HL", self.robot.legs["HL"].ankle_idx),
                ("HR", self.robot.legs["HR"].ankle_idx)
            ]:
                p.changeDynamics(
                    self.robot_id,
                    ankle_idx,
                    lateralFriction=1.5,  # High friction for feet
                    spinningFriction=0.5,
                    rollingFriction=0.5,
                    frictionAnchor=1
                )
            return self.robot

        return self.robot

    def reset(self):
        p.resetSimulation()
        self.load_ground_plane()

    def step(self):
        pos = self.controller.step()
        p.stepSimulation()

        # self.renderer.update()
        # self.renderer.visualize_com(com)

    def clean(self):
        p.disconnect()





