import time
from optparse import Option
import numpy as np
from typing import Tuple, List, Dict, Any, Optional
import pybullet as p
from anyio import sleep

from .parameters import TimeSteppingParams, SolverParameters
from .robot import Robot, JointInfo
from .renderer import Renderer

class Simulation:
    def __init__(
            self,
            solver_iterations: int=SolverParameters.num_solver_iterations,
            time_step: float=TimeSteppingParams.fixed_timestep,
            gui_mode: int=p.GUI,
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


    def connect(self):
        """ Establish connection to physics server"""
        self.phys_client = p.connect(self.gui_mode)
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        p.setPhysicsEngineParameter(numSolverIterations=self.solver_iterations)
        p.setTimeStep(self.time_step)


    def load_ground_plane(self):
        import pybullet_data
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.ground_id = p.loadURDF("plane.urdf")

    def load_robot(self, urdf_path: str, start_pos: List[float]):
        self.robot_id = p.loadURDF(urdf_path, start_pos)
        self.robot = Robot(self.robot_id, self.phys_client)
        self.renderer = Renderer(robot=self.robot, physics_client=self.phys_client)
        self.robot.set_standing_pose()

    def reset(self):
        p.resetSimulation()
        self.load_ground_plane()

    def step(self):
        p.stepSimulation()
        self.renderer.update()
        # self.renderer.visualize_com(com)

    def clean(self):
        p.disconnect()





