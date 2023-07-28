from tkinter import *
import asyncio

from tkinter import *
import asyncio
import os
import signal

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

class JointControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Joint Control")
        
        self.create_sliders()
        self.create_buttons()
    
    def create_sliders(self):
        self.bj_slider = self.create_slider("BJ", -180, 180)
        self.sj_slider = self.create_slider("SJ", -180, 180)
        self.ej_slider = self.create_slider("EJ", -180, 180)
        self.w1j_slider = self.create_slider("W1J", -180, 180)
        self.w2j_slider = self.create_slider("W2J", -180, 180)
        self.w3j_slider = self.create_slider("W3J", -180, 180)
        
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
    jc_app = JointControlApp(root)
    robot = await connect()
    arm = Arm.from_robot(robot, "owl-arm-remote:owl-robot")
    
    def get_js_values():
        return JointPositions(values=[
            jc_app.bj_slider.get(),
            jc_app.sj_slider.get(),
            jc_app.ej_slider.get(),
            jc_app.w1j_slider.get(),
            jc_app.w2j_slider.get(),
            jc_app.w3j_slider.get()
        ])

    def send_js():
        values = get_js_values()
        return asyncio.ensure_future(set_js(arm,values))

    def set_home():
        jc_app.bj_slider.set(0)
        jc_app.sj_slider.set(0)
        jc_app.ej_slider.set(0)
        jc_app.w1j_slider.set(0)
        jc_app.w2j_slider.set(0)
        jc_app.w3j_slider.set(0)

        send_js()
    
    home_button = Button(jc_app.button_frame, text="Home", command=set_home)
    home_button.pack(side=LEFT)
    set_button = Button(jc_app.button_frame, text="Set", command=send_js)
    set_button.pack(side=LEFT)
    
    await run_tk(root)

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main())
