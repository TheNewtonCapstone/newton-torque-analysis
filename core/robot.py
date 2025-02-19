import pybullet as p
import numpy as np
from typing import Tuple, List, Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum
import pprint
from numpy import ndarray
# from pybullet_envs.deep_mimic.env.testLaikago import force


# import pybullet_examples.biped2d_pybullet


@dataclass
class JointChain:
    """ """
    robot_id: int
    phys_client: int
    name: str # name of the joint chain (fr, fl, br, bl)

    # indices of the joints in the chain, these are added for quick lookup
    haa_idx: int
    hfe_inx: int
    kfe_idx: int
    ankle_idx: int # if we are talking about a joint then ankle and if we are talking about a link then foot

    joint_names: List[str] # list of joint names in the chain
    link_names: List[str] # list of link names in the chain

    # for perfomance we include world transformation of the joints
    world_joint_pos: Dict[int, np.ndarray]
    world_joint_orn: Dict[int, np.ndarray]
    link_masses: Dict[int, float]

    def update_link_masses(self, id: int, phys_client:int):
        link_idx = [self.haa_idx, self.hfe_inx, self.kfe_idx, self.ankle_idx]
        for i in link_idx:
            info = p.getDynamicsInfo(id, i, physicsClientId=phys_client)
            self.link_masses[i] = info[0]
    def update_transform(self, id: int, phys_client:int):
        joint_idx = [self.haa_idx, self.hfe_inx, self.kfe_idx, self.ankle_idx]
        states = p.getLinkStates(
            id,
            joint_idx,
            physicsClientId=phys_client
        )

        for i, state in zip(joint_idx, states):
            self.world_joint_pos[i] = np.array(state[0])
            self.world_joint_orn[i] = np.array(state[1])

    def set_joint_pos(self, pose: List[float])->None:
        print(pose)
        print([self.haa_idx, self.hfe_inx, self.kfe_idx])
        p.setJointMotorControlArray(
            self.robot_id,
            jointIndices=[self.haa_idx, self.hfe_inx, self.kfe_idx],
            controlMode=p.POSITION_CONTROL,
            targetPositions=pose,
            forces=[2.5, 2.5, 2.5],
            physicsClientId=self.phys_client
        )
    def get_joint_pos(self, joint_name: str)->np.ndarray:
        idx = self.joint_names.index(joint_name)
        return self.world_joint_pos[idx]
    def get_joint_orn(self, joint_name: str)->np.ndarray:
        idx = self.joint_names.index(joint_name)
        return self.world_joint_orn[idx]
    def get_joint_pose(self, joint_name: str)->Tuple[np.ndarray, np.ndarray]:
        idx = self.joint_names.index(joint_name)
        return self.world_joint_pos[idx], self.world_joint_orn[idx]

    def get_joint_poses(self)->Tuple[Dict[int, np.ndarray], Dict[int, np.ndarray]]:
        return self.world_joint_pos, self.world_joint_orn







class JointType(int, Enum):
    REVOLUTE = 0
    PRISMATIC = 1
    SPHERICAL = 2
    PLANAR = 3
    FIXED = 4
    POINT2POINT = 5
    HINGE = 6
    SLIDER = 7
    UNIVERSAL = 8
    GEAR = 9
    CONE_TWIST = 10
    D6 = 11
    MAX_JOINT_TYPE = 12

@dataclass
class JointState:
    position: float
    velocity: float
    reaction_forces: Tuple[float, float, float]
    applied_torque: float
@dataclass
class LinkState:
    mass: float
    world_pos: Tuple[float, float, float]
    world_orientation: Tuple[float, float, float, float]
    local_inertial_pos: Tuple[float, float, float]
    local_inertial_orn: Tuple[float, float, float, float]
    world_frame_pos: Tuple[float, float, float]
    world_link_frame_orn: Tuple[float, float, float, float]
    world_link_linear_velocity: Tuple[float, float, float]
    world_link_angular_velocity: Tuple[float, float, float]


@dataclass
class JointInfo:
    joint_index: int
    joint_name: str
    joint_type: JointType

    q_index: int #
    u_index: int
    flags: int
    joint_damping: float # specified in URDF
    joint_friction: float # specified in URDF

    joint_lower_limit: float
    joint_upper_limit: float
    joint_max_force: float
    joint_max_velocity: float
    link_name: str
    joint_axis: Tuple[float, float, float]
    parent_frame_pos: Tuple[float, float, float]
    parent_frame_orn: Tuple[float, float, float, float]
    parent_index: int

class Robot:
    def __init__(self, robot_id: int, physics_client: int):
        self.base_mass: Optional[float] = None
        self.id: int = robot_id
        self.num_joints: int = p.getNumJoints(robot_id, physicsClientId=physics_client)
        self.physics_client:int  = physics_client
        self.joints: Dict[str, JointInfo] = {}
        self.links: Dict[str, LinkState] = {}
        self.control_mode: int = p.POSITION_CONTROL
        self.ordered_jts_idx: List[int] = []
        self.legs: Optional[Dict[str, JointChain]] = {}
        self.reset()

    def reset(self):
        self.init_joints()
        self.init_leg_chains()

    def init_joints(self)->None:
       for i in range(self.num_joints):
           info = p.getJointInfo(self.id, i, physicsClientId=self.physics_client)
           joint_name = info[1].decode('utf-8')
           self.ordered_jts_idx.append(info[0])
           self.joints[joint_name] = JointInfo(
               joint_index=info[0],
               joint_name=joint_name,
               joint_type=JointType(info[2]),
               q_index=info[3],
               u_index=info[4],
               flags=info[5],
               joint_damping=info[6],
               joint_friction=info[7],
               joint_lower_limit=info[8],
               joint_upper_limit=info[9],
               joint_max_force=info[10],
               joint_max_velocity=info[11],
               link_name=info[12].decode('utf-8'),
               joint_axis=info[13],
               parent_frame_pos=info[14],
               parent_frame_orn=info[15],
               parent_index=info[16]
           )

    def init_leg_chains(self):
        prefixes = ["FL", "FR", "HL", "HR"]
        for prefix in prefixes:
            haa_name = f"{prefix}_HAA"
            hfe_name = f"{prefix}_HFE"
            kfe_name = f"{prefix}_KFE"
            ankle_name = f"{prefix}_ANKLE"

            self.legs[prefix] = JointChain(
                robot_id=self.id,
                phys_client=self.physics_client,
                name=prefix,
                haa_idx=self.joints[haa_name].joint_index,
                hfe_inx=self.joints[hfe_name].joint_index,
                kfe_idx=self.joints[kfe_name].joint_index,
                ankle_idx=self.joints[ankle_name].joint_index,
                joint_names=[haa_name, hfe_name, kfe_name, ankle_name],
                link_names=[
                    f"{prefix}_SHOULDER",
                    f"{prefix}_UPPER_LEG",
                    f"{prefix}_LOWER_LEG",
                    f"{prefix}_FOOT"
                ],
                world_joint_pos={},
                world_joint_orn={},
                link_masses = {}
            )
            self.legs[prefix].update_transform(self.id, self.physics_client)
            self.legs[prefix].update_link_masses(self.id, self.physics_client)



    def get_base_pose(self)->Tuple[np.ndarray, np.ndarray]:
        """ Return both position and orientation of the robot"""
        pos, orn = p.getBasePositionAndOrientation(self.id, physicsClientId=self.physics_client)
        return np.array(pos), np.array(orn)

    def get_joint_state(self, joint_name: str)-> JointState:
        """ Return the state of the joint"""
        joint_index = self.joints[joint_name].joint_index
        state = p.getJointState(self.id, joint_index, physicsClientId=self.physics_client)
        return JointState(position=state[0], velocity=state[1], reaction_forces=state[2], applied_torque=state[3])

    def set_joint_position(self, joint_name: str, position: float)->None:
        """ Set the position of the joint"""
        joint_index = self.joints[joint_name].joint_index
        p.setJointMotorControl2(self.id, joint_index, controlMode=self.control_mode, targetPosition=position, physicsClientId=self.physics_client)
    def reset_joint_state(self, joint_name: str, position: float)->None:
        """ Reset the position of the joint"""
        joint_index = self.joints[joint_name].joint_index
        p.resetJointState(self.id, joint_index, targetValue=position, targetVelocity=0, physicsClientId=self.physics_client)
    def get_link_state(self, link_name: str)-> LinkState:
        """ Return both position and orientation of the link"""
        link_index = self.joints[link_name].joint_index
        state = p.getLinkState(self.id, link_index, physicsClientId=self.physics_client)
        return LinkState(
            mass=0, # HACK
            world_pos=state[0],
            world_orientation=state[1],
            local_inertial_pos=state[2],
            local_inertial_orn=p.getEulerFromQuaternion(state[3]),
            world_frame_pos=state[4],
            world_link_frame_orn=p.getEulerFromQuaternion(state[5]),
            world_link_linear_velocity=state[6],
            world_link_angular_velocity=state[7]
        )

    def get_joint_names(self)->List[str]:
        """ Return the names of the joints"""
        return list(self.joints.keys())

    def get_link_names(self)->List[str]:
        """ Return the names of the links"""
        return [joint.link_name for joint in self.joints.values()]

    def print_joint_info(self)->None:
        pprint.PrettyPrinter(indent=4).pprint(self.joints)

    def get_com(self) -> np.ndarray:
        """
        Calculate the center of mass of the robot in world coordinates
            R = SUM(mi*ri)/M
        """

        total_mass = 0
        weighted_pos = np.zeros(3)

        # get base mass and position of base
        info = p.getDynamicsInfo(self.id, -1, physicsClientId=self.physics_client)
        base_mass = info[0]
        base_pos, base_orn = p.getBasePositionAndOrientation(self.id, physicsClientId=self.physics_client)

        total_mass += base_mass
        weighted_pos += np.array(base_pos) * base_mass

        for leg in self.legs.values():
            leg.update_transform(self.id, self.physics_client)
            for joint_idx in [leg.haa_idx, leg.hfe_inx, leg.kfe_idx, leg.ankle_idx]:
                mass = leg.link_masses[joint_idx]
                pos = leg.world_joint_pos[joint_idx]
                total_mass += mass
                weighted_pos += np.array(pos) * mass

        # calculate the center of mass
        return weighted_pos / total_mass if total_mass > 0 else np.zeros(3)

    def get_contact_points(self):
        """ Get the contact points of the robot with the ground"""
        info = p.getContactPoints(self.id, physicsClientId=self.physics_client)
        print("__________________________________________")
        pprint.PrettyPrinter(indent=4).pprint(info)


    def set_standing_pose(self):
        """ Set the robot to a standing pose"""
        standing_pose = [
            [1, 0.5, -1.0],
            [-1, 0.5, -1.0],
            [0, 0.5, -1.0],
            [0, 0.5, -1.0]]
        for i, leg in enumerate(self.legs):
            self.legs[leg].set_joint_pos(standing_pose[i])














