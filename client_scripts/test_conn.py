import asyncio
import os

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions

from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions

async def connect():
    secret = os.getenv('OWL_VIAM_SECRET')
    if secret is None:
        raise ValueError("Environment variable OWL_VIAM_SECRET is not set.")
    
    creds = Credentials(
        type='robot-location-secret',
        payload=secret
    )
    opts = RobotClient.Options(
        refresh_interval=0,
        dial_options=DialOptions(credentials=creds)
    )
    return await RobotClient.at_address('owl-robot-dev-main.tzlna5qa2z.viam.cloud', opts)


async def main():
    robot = await connect()

    print('Resources:')
    print(robot.resource_names)
    
    my_arm_component = Arm.from_robot(robot, "owl-arm-remote:owl-robot")
    my_arm_end_position = await my_arm_component.get_end_position()
    print(f"myArm get_end_position return value: {my_arm_end_position}")

    my_arm_joint_positions = await my_arm_component.get_joint_positions()
    print(f"myArm get_joint_positions return value: {my_arm_joint_positions}")

    await robot.close()

if __name__ == '__main__':
    asyncio.run(main())
