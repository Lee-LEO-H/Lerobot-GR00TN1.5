# ~/isaacsim/python.sh sim_scripts/isaacsim_scene.py --receiveAction=True --sendObs=True

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
# import isaacsim.robot_motion.motion_generation as mg
# from isaacsim.storage.native import get_assets_root_path
# from isaacsim.robot.manipulators.manipulators import SingleManipulator
# from isaacsim.robot.manipulators.grippers import ParallelGripper
# import isaacsim.robot.manipulators.controllers as manipulators_controllers
# from isaacsim.robot.manipulators.grippers import ParallelGripper
# from isaacsim.core.prims import Articulation
# import threading
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
import numpy as np
from isaacsim.core.prims import XFormPrim
from isaacsim.core.prims import RigidPrim
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from observation_socket import ObservationSender
from image_socket import ImageSender
from articulation_socket import ActionReceiverThread, ArticulationSender
import time
import argparse
import os
argparser = argparse.ArgumentParser()
argparser.add_argument(
    "--imgStream",
    help="True for image stream, False for not",
    default="False"
)
argparser.add_argument(
    "--receiveAction",
    help="True for receiver Action, False for not",
    default="False"
)
argparser.add_argument(
    "--sendAction",
    help="True for send Action, False for not",
    default="False"
)
argparser.add_argument(
    "--sendObs",
    help="True for send Observation, False for not",
    default="False"
)
argparser.add_argument(
    "--all",
    help="True for starting all, False for not",
    default="False"
)
args = argparser.parse_args()

# create the world 
my_world = World(stage_units_in_meters=1.0)

# add grasp_test.usd to the world 改为相对路径
grasp_test_USD_path = os.path.join(os.path.dirname(__file__), "isaac_models", "grasp_test.usd")
# grasp_test_USD_path = "~/isaacsim/isaac_models/grasp_test.usd"
# grasp_test_USD_path ='/home/b760m/isaacsim/lerobot_sim/grasp_test.usd'
add_reference_to_stage(usd_path=grasp_test_USD_path, prim_path="/World")
my_so101 = my_world.scene.add(
    Robot(
        prim_path="/World/so101_new_calib_02", 
        name="my_so101", 
        scale=np.array([1, 1, 1])
    )
)
# add table to the world
shop_table = my_world.scene.add(
    XFormPrim(
        prim_paths_expr="/World/Shop_Table", 
        name="shop_table", 
        scales=np.array([[1., 1., 1.]])
    )
)
# add cube to the world
cube = my_world.scene.add(
    RigidPrim(
        name="dex_cube1",
        prim_paths_expr="/World/dex_cube_instanceable"
    )
)
# add another cube to the world
cube2 = my_world.scene.add(
    RigidPrim(
        name="dex_cube2",
        prim_paths_expr="/World/dex_cube_instanceable_01"
    )
)
pen = my_world.scene.add(
    RigidPrim(
        name="pen",
        prim_paths_expr="/World/pen"
    )
)

cup = my_world.scene.add(
    RigidPrim(
        name="cup",
        prim_paths_expr="/World/cup"
    )
)

# add camera1 to the world
camera_wrist = Camera(
    prim_path="/World/so101_new_calib_02/gripper_link/Camera_wrist",
    name="Camera_wrist",
    frequency = 30,
    resolution=(640, 480)
)
# add camera2 to the world
camera_front = Camera(
    prim_path="/World/Camera_front",
    name="Camera_front",
    frequency = 30,
    resolution=(640, 480)
)

my_world.reset()
# get the articulation controller of the so101
so101_controller = my_so101.get_articulation_controller()
my_so101.initialize()
camera_wrist.initialize()
camera_front.initialize()

# check information of so101
my_so101_num_dof = my_so101.num_dof
my_so101_num_bodies = my_so101.num_bodies
print("num of dof:"+str(my_so101.num_dof))
print("num of links:"+str(my_so101.num_bodies))
print("type of flexiv_arm :"+str(type(my_so101)))
print("type of flexiv_arm :"+str(type(my_so101)))
print("successfully:"+str(my_so101.handles_initialized))
print("current state of joints:",np.array(my_so101.get_joints_state().positions))

def create_image_sender(host,port):
    image_sender = ImageSender(host=host, port=port)
    if image_sender.socket is None:
        print("发送端启动失败。")
        return None
    else:
        print("发送端启动成功。")
        return image_sender
def get_camera_wrist_image():
    so101_image = camera_wrist.get_rgb()
    image_data_np = np.asarray(so101_image, dtype=np.uint8)
    if image_data_np.shape[0] > 0 and image_data_np.shape[1] > 0:
        return image_data_np
    else:
        return np.zeros((480, 640, 3), dtype=np.uint8)
def get_camera_front_image():
    so101_image = camera_front.get_rgb()
    image_data_np = np.asarray(so101_image, dtype=np.uint8)
    if image_data_np.shape[0] > 0 and image_data_np.shape[1] > 0:
        return image_data_np
    else:
        return np.zeros((480, 640, 3), dtype=np.uint8)
def send_camera_wrist_image():
    image_sender.send_image(get_camera_wrist_image())
def send_camera_front_image():
    image_sender.send_image(get_camera_front_image())
    
# create image sender
if args.imgStream == "True" or args.all == "True":
    image_sender = create_image_sender(host='127.0.0.1', port=65432)
    
# create articulation receiver
if args.receiveAction == "True" or args.all == "True":
    HOST = '127.0.0.1'
    PORT = 65433
    DOF = 6 # 6轴关节角度
    receiver_thread = ActionReceiverThread(HOST, PORT,DOF)
    receiver_thread.daemon = True 
    receiver_thread.start()
    time.sleep(1)
def receive_so101_action():
    action = receiver_thread.get_latest_action()
    if action is not None: 
        if len(action) == my_so101_num_dof:
            return action
    else:
        return np.zeros(my_so101_num_dof,dtype=np.float32)
def apply_so101_action():
    so101_controller.apply_action(ArticulationAction(joint_positions=receive_so101_action()))

# create articulation sender
if args.sendAction == "True" or args.all == "True":
    articulation_sender = ArticulationSender(host='127.0.0.1', port=65434)
# create observation sender
if args.sendObs == "True" or args.all == "True":
    observation_sender = ObservationSender(host='127.0.0.1', port=65435, image_names=["wrist","front"])

# def articulation_send_threading():
#     while True:
#         articulation_sender.send_array(np.array(my_so101.get_joints_state().positions))
#         time.sleep(0.1)
# if args.sendAction == "True":
#     sender_thread = threading.Thread(target=articulation_send_threading)
#     sender_thread.daemon = True
#     sender_thread.start()  
# main



def main():
    if args.receiveAction == "True" or args.all == "True":
        apply_so101_action()
    # send so101 articulation action
    if args.sendAction == "True" or args.all == "True":
        articulation_sender.send_array(np.array(my_so101.get_joints_state().positions))
    # send image
    if args.imgStream == "True" or args.all == "True":
        image_sender.send_image(get_camera_wrist_image())
    # send observation
    if (args.sendObs == "True" or args.all == "True")and observation_sender.connect():
        articulation_data = np.array(my_so101.get_joints_state().positions)
        image_data_wrist = get_camera_wrist_image()
        image_data_front = get_camera_front_image()
        images = [image_data_wrist, image_data_front]
        observation_sender.send_packed_data(np.degrees(articulation_data), images)
# simulation loop
reset_needed = False
try:
    while simulation_app.is_running():
        my_world.step(render=True)
        if my_world.is_stopped() and not reset_needed:
            reset_needed = True
        if my_world.is_playing():
            if reset_needed:
                my_world.reset()
                so101_controller.initialize()
                camera_wrist.initialize()
                camera_front.initialize()   
                # my_controller.reset()
                reset_needed = False
            main()
except KeyboardInterrupt:
    print("Simulation 被用户中断。")
finally:
    # 场景发送端关闭

    if observation_sender:
        observation_sender.close()
    # 场景接收端关闭（接收动作）
    if receiver_thread:
        receiver_thread.stop()

simulation_app.close()
