from tkinter import *
import asyncio

from tkinter import *
import asyncio
import os
import math
import numpy as np

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions

from viam.components.arm import Arm, JointPositions, Pose
# from viam.proto.component.arm import JointPositions, Pose

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

class PoseControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Joint Control")
        
        self.create_sliders()
        self.create_buttons()

        self.x_slider.set(0)
        self.y_slider.set(-250)
        self.z_slider.set(300)
        self.roll_slider.set(180)
        self.pitch_slider.set(0)
        self.yaw_slider.set(0)
    
    def create_sliders(self):
        self.x_slider = self.create_slider("X", -500, 500)
        self.y_slider = self.create_slider("Y", -500, 0)
        self.z_slider = self.create_slider("Z", 0, 600)
        self.roll_slider = self.create_slider("Roll", -180, 180)
        self.pitch_slider = self.create_slider("Pitch", -180, 180)
        self.yaw_slider = self.create_slider("Yaw", -180, 180)

    def create_slider(self, label_text, min_val, max_val):
        slider_frame = Frame(self.root)
        slider_frame.pack(pady=10)
        
        label = Label(slider_frame, text=label_text)
        label.pack(side=LEFT)
        
        slider = Scale(slider_frame, from_=min_val, to=max_val, orient=HORIZONTAL, length=200)
        slider.pack(side=LEFT)
        
        return slider
    
    def create_buttons(self):
        self.button_frame = Frame(self.root)
        self.button_frame.pack(pady=20)
        

async def run_tk(root, interval=0.05):
    '''
    Run a tkinter app in an asyncio event loop.
    '''
    try:
        while True:
            root.update()
            await asyncio.sleep(interval)
    except TclError as e:
        if "application has been destroyed" not in e.args[0]:
            raise

async def set_js(arm,values):
        await arm.move_to_joint_positions(values)


async def main():
    root = Tk()
    pc_app = PoseControlApp(root)
    robot = await connect()
    arm = Arm.from_robot(robot, "owl-arm-remote:owl-robot")
    
    def get_pose():
        roll = pc_app.roll_slider.get()
        pitch = pc_app.pitch_slider.get()
        yaw = pc_app.yaw_slider.get()

        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

        o_x, o_y, o_z, theta = to_orientation_vector(roll, pitch, yaw)

        return Pose(x=pc_app.x_slider.get(), y=pc_app.y_slider.get(), z=pc_app.z_slider.get(),
                    o_x=o_x, o_y=o_y, o_z=o_z, theta=theta)
    def send_pose():
        pose = get_pose()
        return asyncio.ensure_future(arm.move_to_position(pose))


    def set_home():
        pc_app.x_slider.set(0)
        pc_app.y_slider.set(-250)
        pc_app.z_slider.set(300)
        pc_app.roll_slider.set(180)
        pc_app.pitch_slider.set(0)
        pc_app.yaw_slider.set(0)

        send_pose()

    def to_orientation_vector(roll, pitch, yaw):
        # Convert roll, pitch, and yaw to Euler angles
        lon = yaw
        lat = pitch
        theta = roll

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

        # Return the components of the orientation vector and the angle theta
        return o_x, o_y, o_z, np.rad2deg(theta)
    
    home_button = Button(pc_app.button_frame, text="Home", command=set_home)
    home_button.pack(side=LEFT)
    set_button = Button(pc_app.button_frame, text="Set", command=send_pose)
    set_button.pack(side=LEFT)
    
    await run_tk(root)

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main())

