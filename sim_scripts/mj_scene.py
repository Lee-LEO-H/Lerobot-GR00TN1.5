"""Mujoco版本：使用lemj环境"""
"""
指令：
python mj_scene.py --receiveAction True --sendObs True
"""
# todo:: 缺少矫正函数"""
import glfw
import mujoco
import mujoco.viewer
import mj_utils
import numpy as np
import time
import argparse
import os
import rerun as rr

# 切换到脚本目录，确保相对路径（如 myscene.xml 和 include 文件）能被正确找到
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

from observation_socket import ObservationSender
from image_socket import ImageSender
from articulation_socket import ActionReceiverThread, ArticulationSender

argparser = argparse.ArgumentParser()
# 通信控制指令
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
argparser.add_argument(
    "--display_data",
    help="True for display data with rerun, False for not",
    default="False"
)
args = argparser.parse_args()

# 模型和场景文件
MODEL_XML_PATH = os.path.join(script_dir, 'mj_models', 'myscene', 'myscene.xml')
model = mujoco.MjModel.from_xml_path(MODEL_XML_PATH)
data = mujoco.MjData(model)

# 初始化 GLFW（隐藏窗口以使用离屏渲染）
glfw.init()
glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
window = glfw.create_window(1200, 900, "mujoco", None, None)
glfw.make_context_current(window)
# 创建渲染场景和上下文
scene = mujoco.MjvScene(model, maxgeom=1000)
context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)

# 关节名称
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# xml内相机名称
# todo:: 修改xml中的相机名为 wrist, front
# CAMERA_NAMES = ["front_cam", "side_cam"]

# mj相机对象
# 检查相机ID是否存在
wrist_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist")
front_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "front")
# side_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "side")

# print(f"Camera IDs - wrist: {wrist_id}, front: {front_id}, side: {side_id}")

camera_wrist = mj_utils.camera(
    name="wrist", 
    model=model,
    data=data,
    scene=scene,
    context=context,
    frequency=30,
    resolution=(640, 480)
)
camera_front = mj_utils.camera(
    name="front", 
    model=model,
    data=data,
    scene=scene,
    context=context,
    frequency=30,
    resolution=(640, 480)
)
# camera_side = mj_utils.camera(
#     name="side_cam", 
#     model=model,
#     data=data,
#     scene=scene,
#     context=context,
#     frequency=30,
#     resolution=(640, 480)
# )

# mj机械臂对象
my_so101 = mj_utils.robot(
    model=model, 
    data=data,
    joint_names=JOINT_NAMES
)

# 检查 Mujoco 模型关节和身体信息（由于模型不止机械臂，不具参考作用）
my_so101_num_dof = 6  # 自由度数量
my_so101_num_bodies = model.nbody  # 刚体数量
print("num of dof:", my_so101_num_dof)
print("num of bodies:", my_so101_num_bodies)
print("type of model:", type(model))
print("type of data:", type(data))
print("current state of joints:", data.qpos.copy())

def randomize_objects(model, data,
                       object_bodies=['pencil_body', 'pen_body'],
                       table_body='table',
                       frac=1/3,
                       object_width_frac=1.0,
                       width_anchor='center',
                       shrink=0.0,
                       place_target_body=None,
                       place_target_range=(0.15, 0.15),
                       rpy_std=(0.0, 0.0, 2*np.pi)):
    """随机化场景中的物体位置和颜色
    frac: 长边区域占全长的比例；
    object_width_frac: 桌面宽度方向取用的比例(0~1)；
    width_anchor: 宽度方向的锚点，可选 'min'、'center'、'max'；
    shrink: 在选定区域内按比例缩小长宽 (0~1)
    object_bodies: 要随机化的物体 body 列表，仅在 table 区域内；
    table_body: 用于计算区域的桌子 body 名称；
    place_target_body: 如果提供，则在其初始 pos 周围按指定范围随机化；
    place_target_range: (dx,dy) 范围，用于 place_target_body 随机化
    rpy_std: 三元组，分别为 roll/pitch/yaw 的最大随机幅度，单位弧度，默认只随机 yaw
    """
    # 重置所有状态
    mujoco.mj_resetData(model, data)

    # 根据场景中 table body 动态计算 x,y 位置范围，仅在长边1/3靠近机械臂一侧
    table_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, table_body)
    # 找到属于 table body 的 geom id，如果没有直接属于 table 的 geom，则查找其子 body
    geom_ids = [i for i, b in enumerate(model.geom_bodyid) if b == table_bid]
    if not geom_ids:
        # 取直接子 body
        children = [i for i, p in enumerate(model.body_parentid) if p == table_bid]
        for c in children:
            geom_ids += [i for i, b in enumerate(model.geom_bodyid) if b == c]
    if not geom_ids:
        # 找不到桌面 geom，退出随机化
        return
    # 取最大的半长半宽
    half_x = max(model.geom_size[g][0] for g in geom_ids)
    half_y = max(model.geom_size[g][1] for g in geom_ids)
    bx, by, _ = model.body_pos[table_bid]
    # 长边方向占比 frac, 并在区域内按 shrink 缩小长宽
    # 计算原始选区
    full_x = 2 * half_x
    full_y = 2 * half_y
    table_xmin = bx - half_x
    table_xmax = bx + half_x
    region_xmin = bx - half_x
    region_len_x = full_x * frac
    region_xmax = region_xmin + region_len_x
    width_frac_clipped = np.clip(object_width_frac, 0.0, 1.0)
    region_len_y = full_y * width_frac_clipped
    if region_len_y <= 0:
        return

    if width_anchor == 'min':
        region_ymin = by - half_y
    elif width_anchor == 'max':
        region_ymin = by + half_y - region_len_y
    else:  # 'center' or any other value
        region_ymin = by - region_len_y / 2

    # Clamp 到桌面宽度范围内
    region_ymin = max(region_ymin, by - half_y)
    region_ymax = min(region_ymin + region_len_y, by + half_y)
    # 如果 clamp 导致范围过小，则退回到桌面边界
    if region_ymax - region_ymin < 1e-6:
        region_ymin = by - half_y
        region_ymax = by + half_y
        region_len_y = full_y

    region_len_y = region_ymax - region_ymin
    # 区域中心
    cx = region_xmin + region_len_x / 2
    cy = region_ymin + region_len_y / 2
    # 缩小后半长度
    half_len_x = (region_len_x * (1 - shrink)) / 2
    half_len_y = (region_len_y * (1 - shrink)) / 2
    xmin = cx - half_len_x
    xmax = cx + half_len_x
    ymin = cy - half_len_y
    ymax = cy + half_len_y
    z = 0.56  # 防止戳穿笔筒

    # 随机化主要对象，在 table 区域内
    for body_name in object_bodies:
        # 根据 body 找到它的 free joint id
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        jids = [j for j, b in enumerate(model.jnt_bodyid) if b == bid]
        if not jids:
            continue
        adr = model.jnt_qposadr[jids[0]]
        # 随机平移
        data.qpos[adr:adr+3] = np.random.uniform((xmin, ymin, z), (xmax, ymax, z))
        # 按 rpy_std 随机旋转
        roll = np.random.uniform(-rpy_std[0]/2, rpy_std[0]/2)
        pitch = np.random.uniform(-rpy_std[1]/2, rpy_std[1]/2)
        yaw = np.random.uniform(-rpy_std[2]/2, rpy_std[2]/2)
        cr, sr = np.cos(roll/2), np.sin(roll/2)
        cp, sp = np.cos(pitch/2), np.sin(pitch/2)
        cy, sy = np.cos(yaw/2), np.sin(yaw/2)
        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        data.qpos[adr+3:adr+7] = np.array([qw, qx, qy, qz])

        # Domain randomization: 随机化pen颜色和尺寸
        try:
            pen_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if pen_bid >= 0:
                # 找到属于pen_body的geom
                for geom_id in range(model.ngeom):
                    if model.geom_bodyid[geom_id] == pen_bid:
                        # 随机化颜色 - 在深色调范围内变化
                        # base_color = np.array([0.1, 0.15, 0.2])  # 基础深蓝色
                        base_color = np.array([0.5, 0.5, 0.5])  # 基础
                        color_variation = 0.5  # 颜色变化范围
                        
                        r_var = np.random.uniform(-color_variation, color_variation)
                        g_var = np.random.uniform(-color_variation, color_variation) 
                        b_var = np.random.uniform(-color_variation, color_variation)
                        
                        new_pen_color = base_color + np.array([r_var, g_var, b_var])
                        new_pen_color = np.clip(new_pen_color, 0.0, 0.8)  # 保持在合理范围
                        
                        # 直接修改geom的rgba
                        model.geom_rgba[geom_id, :3] = new_pen_color
                        model.geom_rgba[geom_id, 3] = 1.0  # 保持不透明
                        
                        # 随机化尺寸 - 在原始尺寸基础上小幅变化
                        original_size = np.array([0.0045, 0.06])  # 原始尺寸
                        size_variation = 0.2  # 20%的尺寸变化
                        
                        size_scale_x = np.random.uniform(1.0 - size_variation, 1.0 + size_variation)
                        size_scale_y = np.random.uniform(1.0 - size_variation, 1.0 + size_variation)
                        
                        new_size = original_size * np.array([size_scale_x, size_scale_y])
                        new_size = np.clip(new_size, [0.003, 0.05], [0.008, 0.1])  # 限制尺寸范围
                        
                        model.geom_size[geom_id, :2] = new_size
                        break
        except Exception as e:
            print(f"Warning: Failed to randomize pen: {e}")
            pass
    
    # Domain randomization: 随机化penholder(笔筒)颜色
    # 笔筒是由多个geom组成的，需要随机化它们的rgba属性
    try:
        pen_holder_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'pen_holder')
        if pen_holder_bid >= 0:
            # 定义笔筒的基础颜色 (白色)
            base_holder_color = np.array([1.0, 1.0, 1.0])
            color_variation = 0.2  # 颜色变化范围
            
            # 生成随机颜色变化
            r_var = np.random.uniform(-color_variation, color_variation)
            g_var = np.random.uniform(-color_variation, color_variation)
            b_var = np.random.uniform(-color_variation, color_variation)
            
            new_holder_color = base_holder_color + np.array([r_var, g_var, b_var])
            new_holder_color = np.clip(new_holder_color, 0.6, 1.0)  # 保持在亮色范围
            
            # 找到属于pen_holder的所有geom并更新颜色
            for geom_id in range(model.ngeom):
                if model.geom_bodyid[geom_id] == pen_holder_bid:
                    model.geom_rgba[geom_id, :3] = new_holder_color
                    model.geom_rgba[geom_id, 3] = 1.0  # 保持不透明
    except:
        pass

    # 如果提供了目标放置点 body，则仅随机化该 body 的 joint qpos
    if place_target_body:
        # 根据 body 找到它的 free joint id
        bid_t = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, place_target_body)
        jids_t = [j for j, b in enumerate(model.jnt_bodyid) if b == bid_t]
        if jids_t:
            adr_t = model.jnt_qposadr[jids_t[0]]
            # 原始位置
            bpos = model.body_pos[bid_t]
            # 随机偏移
            dx = np.random.uniform(-place_target_range[0], place_target_range[0])
            dy = np.random.uniform(-place_target_range[1], place_target_range[1])
            # 高度保持到桌面上方
            dz = bpos[2] + 0.0
            data.qpos[adr_t:adr_t+3] = np.array([bpos[0] + dx, bpos[1] + dy, dz])
            # 仅绕 Z 轴随机旋转
            yaw = np.random.uniform(0, 2*np.pi)
            qw = np.cos(yaw/2)
            qz = np.sin(yaw/2)
            data.qpos[adr_t+3:adr_t+7] = np.array([qw, 0, 0, qz])

    # 把更新过的状态向前推一次，以便渲染生效
    mujoco.mj_forward(model, data)

def create_image_sender(host,port):
    """实时画面：rgb>socket>record"""
    image_sender = ImageSender(host=host, port=port)
    if image_sender.socket is None:
        print("发送端启动失败。")
        return None
    else:
        print("发送端启动成功。")
        return image_sender
    
def get_camera_wrist_image():
    """实时画面：场景>rgb"""
    so101_image = camera_wrist.get_rgb()
    image_data_np = np.asarray(so101_image, dtype=np.uint8)
    if image_data_np.shape[0] > 0 and image_data_np.shape[1] > 0:
        return image_data_np
    else:
        return np.zeros((480, 640, 3), dtype=np.uint8)
    
def get_camera_front_image():
    """实时画面：场景>rgb"""
    so101_image = camera_front.get_rgb()
    image_data_np = np.asarray(so101_image, dtype=np.uint8)
    if image_data_np.shape[0] > 0 and image_data_np.shape[1] > 0:
        return image_data_np
    else:
        return np.zeros((480, 640, 3), dtype=np.uint8)

def get_camera_side_image():
    """实时画面：场景>rgb"""
    so101_image = camera_side.get_rgb()
    image_data_np = np.asarray(so101_image, dtype=np.uint8)
    if image_data_np.shape[0] > 0 and image_data_np.shape[1] > 0:
        return image_data_np
    else:
        return np.zeros((480, 640, 3), dtype=np.uint8)
    
def send_camera_wrist_image():
    """实时画面：rgb>socket>record"""
    image_sender.send_image(get_camera_wrist_image())

def send_camera_front_image():
    """实时画面：rgb>socket>record"""
    image_sender.send_image(get_camera_front_image())

def send_camera_side_image():
    """实时画面：rgb>socket>record"""
    image_sender.send_image(get_camera_side_image())


image_sender = None
receiver_thread = None
articulation_sender = None
observation_sender = None
last_obs_connect_log = 0.0
    
def receive_so101_action():
    """遥控关节：遥控>action"""
    if receiver_thread is None:
        return np.zeros(my_so101_num_dof, dtype=np.float32)

    action = receiver_thread.get_latest_action()

    if action is not None and len(action) == my_so101_num_dof:
        return np.asarray(action, dtype=np.float32)

    return np.zeros(my_so101_num_dof, dtype=np.float32)
    
def teleop_offset(action: list[float]):
    """
    关节偏置量
    -0.0612
    0.0175
    0.112
    0.0447
    0.119
    0.948
    """
    action[0] -= -0.0612
    action[1] -= 0.0175
    action[2] -= 0.112
    action[3] -= 0.0447
    action[4] -= 0.35
    # 夹爪模型安装孔位与leader不同，但与follower相同，要再偏置0.795
    # action[5] -= 0.948 - 0.795
    action[5] -= 0.948 - 0.6

def teleop_limit(action: list[float]):
    """
    关节动作限制
    """
    # 机械臂关节限制
    # action[0] = np.clip(action[0], -2.9671, 2.9671)
    # action[1] = np.clip(action[1], -1.8326, 1.3089)
    # action[2] = np.clip(action[2], -2.9671, 2.9671)
    # action[3] = np.clip(action[3], -3.3161, 0.0873)
    # action[4] = np.clip(action[4], -6.2832, 6.2832)
    # 夹爪关节限制
    # action[5] = np.clip(action[5], -0., 0.8)
    pass

def apply_so101_action():
    """遥控关节：action>场景"""
    # so101_controller.apply_action(ArticulationAction(joint_positions=receive_so101_action()))
    action_received_from_teleop = receive_so101_action()
    # teleop_offset(a)
    print(f"接收的action: {action_received_from_teleop[3]}")
    # teleop_limit(a)
    # 夹爪关节缩放到模型范围

    action_received_from_teleop[0] -= -0.0612
    action_received_from_teleop[1] -= 0.0175
    action_received_from_teleop[2] -= 0.112
    action_received_from_teleop[3] -= 0.0447

    # a[0] *= 1.5
    # a[1] *= 1.5
    # a[2] *= 1.5
    # a[3] *= 1.5

    # 将 a[4] 从 [-1.74, 1.74] 映射到 [-2.74, 2.84]
    action_received_from_teleop[4] = (action_received_from_teleop[4] - (-1.74)) * (2.84 - (-2.74)) / (1.74 - (-1.74)) + (-2.74)

    # 先平移到0~1.64；再缩放到0~1.914；最后平移到-0.174~1.74
    action_received_from_teleop[5] = (action_received_from_teleop[5] - 0.01) * (1.74 - (-0.174)) / (1.64 - 0.01) + (-0.174)
    # a[5] *= (1.74-(-0.174)) / (1.38 - (-0.33))  # 夹爪关节缩放到模型范围

    print(f"发送的action: {action_received_from_teleop[3]}")

    data.ctrl[:] = action_received_from_teleop

# 创建图像发送器 image sender
if args.imgStream == "True" or args.all == "True":
    """实时画面：场景>rgb>socket>record"""
    image_sender = create_image_sender(host='127.0.0.1', port=65432)
    
# 创建遥控关节接受器 articulation receiver
if args.receiveAction == "True" or args.all == "True":
    """遥控关节：遥控>action>场景"""
    HOST = '127.0.0.1'
    PORT = 65433
    DOF = 6 # 6轴关节角度
    receiver_thread = ActionReceiverThread(HOST, PORT,DOF)
    receiver_thread.daemon = True 
    receiver_thread.start()
    time.sleep(1)

# 创建实时关节发送器 articulation sender
if args.sendAction == "True" or args.all == "True":
    articulation_sender = ArticulationSender(host='127.0.0.1', port=65434)

# 创建实时综合数据发送器 observation sender
if args.sendObs == "True" or args.all == "True":
    # 根据可用摄像头自动配置 image_names
    # 检查哪些摄像头ID有效来决定配置
    available_cameras = []
    if wrist_id >= 0:
        available_cameras.append("wrist")
    if front_id >= 0:
        available_cameras.append("front")
    # if side_id >= 0:
    #     # available_cameras.append("side")
    #     pass
    
    print(f"Available cameras for observation sender: {available_cameras}")
    observation_sender = ObservationSender(host='127.0.0.1', port=65435, image_names=available_cameras)

def communication_step():
    """单步通信"""
    global last_obs_connect_log

    if args.receiveAction == "True" or args.all == "True":
        """遥控关节：action>场景"""
        apply_so101_action()

    if args.sendObs == "True" or args.all == "True":
        if observation_sender is None:
            return

        if not observation_sender.is_connected():
            if not observation_sender.connect():
                now = time.time()
                if now - last_obs_connect_log > 2.0:
                    print("ObservationSender 等待接收端连接中...")
                    last_obs_connect_log = now
                return

        articulation_data = np.array(my_so101.get_joints_state())

        images = []
        if wrist_id >= 0:
            images.append(get_camera_wrist_image())
        if front_id >= 0:
            images.append(get_camera_front_image())

        if not observation_sender.send_packed_data(np.degrees(articulation_data), images):
            observation_sender.close()

        # 展示图像发送
        if args.display_data == "True":
            # 记录关节角度数据到 rerun
            joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
            for i, (joint_name, joint_angle) in enumerate(zip(joint_names, np.degrees(articulation_data))):
                rr.log(f"robot/joints/{joint_name}", rr.Scalar(joint_angle), static=False)
            
            # 创建关节角度的时间序列图
            rr.log("plots/joint_angles", rr.SeriesLine(
                color=[255, 0, 0], name="Joint Angles"
            ), static=True)
            
            # 记录图像数据到 rerun
            if wrist_id >= 0 and len(images) > 0:
                # 记录腕部相机图像
                wrist_img = images[0]
                rr.log("cameras/wrist", rr.Image(wrist_img), static=False)
            
            if front_id >= 0 and len(images) > 1:
                # 记录前方相机图像
                front_img = images[1]
                rr.log("cameras/front", rr.Image(front_img), static=False)
            
            # 记录动作数据到 rerun (如果有接收到动作)
            if args.receiveAction == "True" or args.all == "True":
                action = receive_so101_action()
                for i, (joint_name, action_val) in enumerate(zip(joint_names, action)):
                    rr.log(f"teleop/action/{joint_name}", rr.Scalar(action_val), static=False)
                
                # 创建动作的时间序列图
                rr.log("plots/action_commands", rr.SeriesLine(
                    color=[0, 255, 0], name="Action Commands"
                ), static=True)
            
            # 记录仿真时间
            rr.log("simulation/time", rr.Scalar(data.time), static=False)


def main():
    """主循环：运行仿真和通信"""
    # 初始化 rerun (如果需要数据展示)
    if args.display_data == "True":
        rr.init("MuJoCo Robot Simulation", spawn=True)
        rr.log("description", rr.TextDocument("Real-time robot simulation with MuJoCo"), static=True)
    
    print("开始仿真循环...")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 固定步长仿真控制
        sim_speed = 1.0
        acc = 0.0
        last_wall = time.time()

        # 使用重置检测（点击 GUI Reset 按钮会将 data.time 重置为 0）
        prev_time = data.time

        try:
            while viewer.is_running():
                now = time.time()
                acc += (now - last_wall) * sim_speed
                last_wall = now
                
                h = model.opt.timestep
                max_catchup = int(10 / h)
                steps = 0
                
                while acc >= h and steps < max_catchup:
                    # 进行仿真步进
                    mujoco.mj_step(model, data)  # type: ignore[attr-defined]
                    acc -= h
                    steps += 1

                    # 检测 GUI Reset：如果 data.time 回退则执行随机化
                    if data.time < prev_time:
                        randomize_objects(
                            model,
                            data,
                            object_bodies=['pencil_body', 'pen_body'],
                            table_body='table',
                            frac=1.3/3,
                            object_width_frac=0.7,
                            width_anchor='min',
                            shrink=0.8,
                            place_target_body='pen_holder',
                            place_target_range=(0.03, 0.03)
                        )
                    prev_time = data.time

                # 刷新显示器
                viewer.sync()

                # 执行通信步骤
                communication_step()

                # 短暂延迟避免CPU占用过高
                time.sleep(0.001)

        except KeyboardInterrupt:
            print("收到中断信号，正在关闭...")

        finally:
            # 清理资源（发送）
            if articulation_sender:
                articulation_sender.close()
            if image_sender:
                image_sender.close()
            if observation_sender:  
                observation_sender.close()
            # todo:: 清理资源（接收）
            glfw.terminate()
            print("仿真结束")

if __name__ == "__main__":
    main()