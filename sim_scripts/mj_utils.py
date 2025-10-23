import mujoco
import numpy as np

class camera:
    def __init__(self, name, model, data, scene, context, frequency=30, resolution=(640, 480)):
        # 创建独立的MjvCamera对象而不是继承
        self.mjv_camera = mujoco.MjvCamera()
        self.mjv_camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        
        # 获取相机ID并检查是否有效
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        if cam_id == -1:
            raise ValueError(f"Camera '{name}' not found in model")
        
        self.mjv_camera.fixedcamid = cam_id
        
        self.name = name
        self.resolution = resolution  # 默认分辨率
        self.frequency = frequency  # 默认频率

        self.model = model
        self.data = data
        self.scene = scene
        self.context = context
        
        print(f"Camera '{name}' initialized with ID: {cam_id}")

    def get_bgr(self, model, data, scene: mujoco.MjvScene, context: mujoco.MjrContext):
        """从指定相机渲染一帧，返回 BGR numpy 图像"""
        viewport = mujoco.MjrRect(0, 0, *self.resolution)
        mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, self.mjv_camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
        mujoco.mjr_render(viewport, scene, context)
        bgr = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)
        depth = np.zeros((self.resolution[1], self.resolution[0]), dtype=np.float64)
        mujoco.mjr_readPixels(bgr, depth, viewport, context)
        return np.flipud(bgr)  # 返回BGR
    
    def get_rgb666(self, model, data, scene: mujoco.MjvScene, context: mujoco.MjrContext):
        """从指定相机渲染一帧，返回 RGB numpy 图像"""
        bgr = self.get_bgr(model, data, scene, context)
        rgb = bgr[:, :, ::-1]  # BGR 转 RGB
        return rgb
    
    def get_rgb(self):
        """从指定相机渲染一帧，返回 RGB numpy 图像"""
        # 确保使用正确的相机
        viewport = mujoco.MjrRect(0, 0, *self.resolution)
        
        # 更新场景
        mujoco.mjv_updateScene(self.model, self.data, mujoco.MjvOption(), None, self.mjv_camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        
        # 渲染场景
        mujoco.mjr_render(viewport, self.scene, self.context)
        
        # 读取像素数据
        img = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)
        depth = np.zeros((self.resolution[1], self.resolution[0]), dtype=np.float64)
        mujoco.mjr_readPixels(img, depth, viewport, self.context)
        
        # MuJoCo默认返回BGR格式，转换为RGB并垂直翻转
        img_rgb = img[:, :, [2, 1, 0]]  # BGR -> RGB
        img_flipped = np.flipud(img_rgb)  # 垂直翻转
        
        return img_flipped
    
class robot:
    def __init__(self, joint_names, model, data):
        self.joint_names = joint_names
        self.model = model
        self.data = data
        self.joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]

    def get_joints_state(self):
        """获取当前关节状态 []*6"""
        return [self.data.qpos[id] for id in self.joint_ids]
