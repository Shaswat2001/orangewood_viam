# owl_viam_server/owl_robot.py

import asyncio
from typing import ClassVar, Mapping, Sequence, Any, Dict, Optional, cast
from viam.components.arm import Arm, JointPositions, Pose as ViamPose
from viam.components.arm import KinematicsFileFormat
from viam.operations import run_with_operation
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName, Vector3
from viam.resource.base import ResourceBase
from viam.resource.types import Model, ModelFamily
from viam.logging import getLogger

LOGGER = getLogger(__name__)

from owl_client import OwlClient
from owl_client.client.Interfaces.utils import Joint, RobotMode, Pose as OwlPose

# from trac_ik_python.trac_ik import IK

import time
import numpy as np

from typing_extensions import Self

class OwlRobot(Arm, Reconfigurable):

    MODEL: ClassVar[Model] = Model(ModelFamily("rdk", "fake"),"owl_robot")

    def __init__(self, name: str):
        # Setup Owl Robot Client
        _robot_ip = 'localhost'
        self.client = OwlClient(_robot_ip)
        while not self.client.is_running():
            LOGGER.info("Waiting for client to start...")
            time.sleep(0.2)
        LOGGER.info("Robot connected...")
        with open('/home/shaswatgarg/owl_viam/owl_viam_server/urdf_6_3_updated.urdf') as f:
            urdf = f.read()
        
        # self.ik_solver = IK("world", "tcp",urdf_string=urdf)
        # self.seed = [0.0] * self.ik_solver.number_of_joints
        self.lower_bound = (-2.094399929046631, -1.600000023841858, -2.5,
                            -1.600000023841858, -1.600000023841858,
                            -3.0999999046325684)
        self.upper_bound = (2.094399929046631, 1.600000023841858, 2.5,
                            1.600000023841858, 2.0999999046325684,
                            3.0999999046325684)
        # self.ik_solver.set_joint_limits(self.lower_bound, self.upper_bound)

        position = self.client.get_tcp()
        self.position = self.to_viam_pose(position)

        self.joint_positions = JointPositions(values=list(np.rad2deg(self.client.get_joint().get_joints())))

        self.is_stopped = True
        super().__init__(name)
        LOGGER.info("Robot Initialized")


    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        arm = cls(config.name)
        # arm.reconfigure(config, dependencies)
        return arm

    # Validates JSON Configuration
    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        pass

    # Handles attribute reconfiguration
    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        pass
    
    async def get_end_position(self, extra: Optional[Dict[str, Any]] = None, **kwargs) -> ViamPose:
        position = self.client.get_tcp()
        self.position = self.to_viam_pose(position)
        return self.position

    @run_with_operation
    async def move_to_position(
        self,
        pose: ViamPose,
        extra: Optional[Dict[str, Any]] = None,
        **kwargs,
    ):
        
        pass
    #     operation = self.get_operation(kwargs)

    #     self.is_stopped = False
    #     self.position = pose
    #     self.seed = np.deg2rad(self.joint_positions.values)
        
    #     joint_positions = self.ik_solver.get_ik(self.seed,self.position.x/1000.0, self.position.y/1000.0, self.position.z/1000.0,1.0,0.0,0.0,0.0)

    #     if joint_positions is None:
    #         raise Exception("No IK solution found")
    #     else:
    #         joint_positions = np.rad2deg(joint_positions)
    #         print(joint_positions)

    #     await self.move_to_joint_positions(JointPositions(values=joint_positions))

    #     # Simulate the length of time it takes for the arm to move to its new position
    #     for x in range(2):
    #         await asyncio.sleep(1)

    #         # Check if the operation is cancelled and, if it is, stop the arm's motion
    #         if await operation.is_cancelled():
    #             await self.stop()
    #             break

    #     self.is_stopped = True

    async def get_joint_positions(self, extra: Optional[Dict[str, Any]] = None, **kwargs) -> JointPositions:
        self.joint_positions = JointPositions(values=list(np.rad2deg(self.client.get_joint().get_joints())))
        return self.joint_positions


    @run_with_operation
    async def move_to_joint_positions(self, positions: JointPositions, extra: Optional[Dict[str, Any]] = None, **kwargs):
        operation = self.get_operation(kwargs)

        goal_positions = list(np.deg2rad(positions.values))
        print("Received Joint Positions: ", goal_positions)
        joint_positions = Joint()
        joint_positions.from_array(goal_positions)

        try:
            self.client.move_to_joint(joint_positions,50,wait=True)
        except Exception as e:
            raise Exception(f"Error moving to joint positions: {e.details()}")

        self.is_stopped = True
        print("Joint Motion Complete")

    async def stop(self, extra: Optional[Dict[str, Any]] = None, **kwargs):
        self.client.move_abort()
        self.is_stopped = True

    async def is_moving(self) -> bool:
        return not self.is_stopped
    
    def to_viam_pose(self, pose: OwlPose) -> ViamPose:

        lon,lat,theta = pose.roll, pose.pitch, pose.yaw

        # Calculate the components of the orientation vector
        default_angle_epsilon = 1e-4
        o_z = np.cos(lat)
        if 1 - np.abs(o_z) > default_angle_epsilon:
            r = np.sin(lat)  # radius of the horizontal circle
            o_x = r * np.cos(lon)
            o_y = r * np.sin(lon)
        else:
            # If we are at the poles, the x and y components are not well-defined, set them to 0
            o_x = 0.0
            o_y = 0.0

        viam_pose = ViamPose(
            x=pose.x,
            y=pose.y,
            z=pose.z,
            o_x=o_x,
            o_y=o_y,
            o_z=o_z,
            theta=np.rad2deg(theta),
        )

        return viam_pose
    
    async def get_kinematics(self, *, timeout: Optional[float] = None):
        print("Getting Kinematics")
        
        with open ('/home/shaswatgarg/owl_viam/owl_viam_server/urdf_6_3_updated.urdf') as f:
            urdf = f.read()
            # convert to bytes
            urdf_bytes = urdf.encode('utf-8')
        
            return (KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF, urdf_bytes)


