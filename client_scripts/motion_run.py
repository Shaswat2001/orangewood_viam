import asyncio

from viam.components.arm import Arm
from viam.components.gripper import Gripper
from viam.proto.common import Geometry, GeometriesInFrame, Pose, PoseInFrame, RectangularPrism, Vector3, WorldState
from viam.proto.component.arm import JointPositions
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.services.motion import MotionClient
import os

async def connect():
    creds = Credentials(
        type='robot-location-secret',
        payload='9b1bzknfu9rjaj06yebmdnrn7bx1ucwddily4lor0273j76n')
    opts = RobotClient.Options(
        refresh_interval=0,
        dial_options=DialOptions(credentials=creds)
    )
    return await RobotClient.at_address('owl-viam-linux-main.cz9i6pmn9t.viam.cloud', opts)


async def main():
    robot = await connect()

    print('Resources:')
    print(robot.resource_names)

    # Access myArm
    my_arm_resource_name = Arm.get_resource_name("owl_robot")
    print(my_arm_resource_name)
    my_arm_component = Arm.from_robot(robot, "owl_robot")
    print(my_arm_component)

    # End Position of myArm
    # my_arm_end_position = await my_arm_component.get_end_position()
    # print(f"myArm get_end_position return value: {my_arm_end_position}")

    # # Joint Positions of myArm
    # my_arm_joint_positions = await my_arm_component.get_joint_positions()
    # print(f"myArm get_joint_positions return value: {my_arm_joint_positions}")

    # Command a joint position move: move the forearm of the arm slightly up 
    # cmd_joint_positions = JointPositions(values=[0, 0, -30.0, 0, 0, 0])
    # await my_arm_component.move_to_joint_positions(positions=cmd_joint_positions)
    # print("moved to joint")
    # # Generate a simple pose move +100mm in the +Z direction of the arm
    # cmd_arm_pose = await my_arm_component.get_end_position()
    # cmd_arm_pose.z += 100.0
    # await my_arm_component.move_to_position(pose=cmd_arm_pose)

    # Access the Motion Service
    motion_service = MotionClient.from_robot(robot, "builtin")
    # print(motion_service)
    # my_arm_component = Arm.from_robot(robot, "owl_robot")
    # print(my_arm_component)
    # my_arm_joint_positions = await my_arm_component.get_joint_positions()
    # print(f"myArm get_joint_positions return value: {my_arm_joint_positions}")

    # cmd_joint_positions = JointPositions(values=[0, 0, -30.0, 0, 0, 0])
    # await my_arm_component.move_to_joint_positions(positions=cmd_joint_positions)

    # Get the pose of myArm from the Motion Service
    # my_arm_motion_pose = await motion_service.get_pose(my_arm_resource_name, "world")
    # print(f"Pose of myArm from the Motion Service: {my_arm_motion_pose}")

    # Add a table obstacle to a WorldState
    # table_origin = Pose(x=-202.5, y=-546.5, z=-19.0)
    # table_dims = Vector3(x=635.0, y=1271.0, z=38.0)
    # table_object = Geometry(center=table_origin, box=RectangularPrism(dims_mm=table_dims))

    # obstacles_in_frame = GeometriesInFrame(reference_frame="world", geometries=[table_object])

    # Create a WorldState that has the GeometriesInFrame included
    world_state = WorldState()

    get_end = await my_arm_component.get_end_position()
    print(get_end)

    # Generate a sample "start" pose to demonstrate motion
    test_start_pose = Pose(x=210.0, y=0.0, z=226.0, o_x=0.7071, o_y=0.0, o_z=-0.7071, theta=0.0)
    test_start_pose_in_frame = PoseInFrame(reference_frame="world", pose=test_start_pose)
    print('Running motive services')
    await motion_service.move(component_name=my_arm_resource_name, destination=test_start_pose_in_frame, world_state=world_state)

    # Don't forget to close the robot when you're done!
    await robot.close()

if __name__ == '__main__':
    asyncio.run(main())


