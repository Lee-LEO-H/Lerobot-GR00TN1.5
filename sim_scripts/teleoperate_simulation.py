# python ./sim_scripts/teleoperate_simulation.py --fps 40 --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.id=my_awesome_leader_arm

import asyncio
import json
import time
import numpy as np
from typing import Dict, Any, Optional, List

import logging
import time
from dataclasses import asdict, dataclass, field
from pprint import pformat

import rerun as rr

from articulation_socket import ArticulationSender
import numpy as np
import time
import draccus

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401

from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    bi_so100_leader,
    gamepad,
    homunculus,
    koch_leader,
    make_teleoperator_from_config,
    so100_leader,
    so101_leader,
)
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import _init_rerun, log_rerun_data

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
    action[5] -= 0.948 - 0.795

@dataclass
class TeleoperateConfig:
    # 为teleop提供默认配置
    teleop: TeleoperatorConfig = field(default_factory=TeleoperatorConfig)
    
    # Limit the maximum frames per second.
    fps: int = 60
    teleop_time_s: float | None = None
    # Display all cameras on screen
    display_data: bool = False
    # ROS2 配置
    joint_state_topic: str = "/joint_states"
    action_topic: str = "/teleop_actions"
    node_name: str = "lerobot_teleop_node"


async def teleop_loop_async(
    teleop: Teleoperator, 
    fps: int, 
    display_data: bool = False, 
    duration: float | None = None,
    sender: ArticulationSender = None
):
    """异步遥操作主循环"""
    start = time.perf_counter()


    # 获取action键名用于显示
    sample_action = teleop.get_action()
    action_keys = list(sample_action.keys())
    display_len = max(len(key) for key in action_keys)
    
    # 添加首次启动提示
    print("🔍 等待接收关节状态数据...")
    joint_state_wait_start = time.time()
    
    while True:
        loop_start = time.perf_counter()
        
        # 获取遥操作action
        action = teleop.get_action()
        print(f"机械臂的action: {action}")
        joint_values = [
            action["shoulder_pan.pos"],
            action["shoulder_lift.pos"],
            action["elbow_flex.pos"],
            action["wrist_flex.pos"],
            action["wrist_roll.pos"],
            action["gripper.pos"],
        ]
        
        # 关节【4】【5】误差补偿
        joint_values[4] += 21
        joint_values[5] += 7.5 
        joint_values = np.deg2rad(joint_values)
        teleop_offset(joint_values)

        # --- 低通滤波处理 ---
        alpha = 0.1  # 平滑系数，可调试
        if not hasattr(teleop_loop_async, "previous_joint_values"): # 若无前值则初始化
            teleop_loop_async.previous_joint_values = joint_values.copy()
        # 滤波: new = alpha * current + (1-alpha) * previous
        joint_values = alpha * np.array(joint_values) + (1 - alpha) * np.array(teleop_loop_async.previous_joint_values)
        teleop_loop_async.previous_joint_values = joint_values.copy()

        # 发送关节数据
        sender.send_array(joint_values)
        print(111)
   
        # 本次循环耗时，用以计算FPS
        loop_s = time.perf_counter() - loop_start
        
        # 显示当前状态信息
        print("\n" + "=" * 50)
        print(f"🤖 遥操作状态监控")
        print("=" * 50)
        
        # 显示遥操作信息
        print(f"\n🎮 遥操作信息(目标帧率{fps}FPS): ")
        # todo:: FPS计算不准确
        print(f"实际FPS: {1/loop_s:.1f}Hz, 延时: {loop_s*1000:.1f}ms")
        print(f"{'ACTION':<{display_len}} | {'VALUE(degree)':>7}")
        for key, value in action.items():
            print(f"{key:<{display_len}} | {value:>7.2f} ")
        
        # 控制循环频率
        dt_s = time.perf_counter() - loop_start
        await asyncio.sleep(max(0, 1/fps - dt_s))
        
        # 检查是否达到运行时间限制
        if duration is not None and time.perf_counter() - start >= duration:
            print(f"⏰ 单轮循环超时，限制: {duration}秒，程序退出。")
            return


@draccus.wrap()
def teleoperate(cfg: TeleoperateConfig):
    """主遥操作函数（整合ROS2）"""
    init_logging()
    logging.info(pformat(asdict(cfg)))
    
    # 初始化sender
    sender = ArticulationSender('127.0.0.1', 65433)
    
    if cfg.display_data:
        _init_rerun(session_name="teleoperation")

    # 初始化遥操作设备
    teleop = make_teleoperator_from_config(cfg.teleop)
    teleop.connect()
    
    async def run_async():
        """异步运行主循环"""
        try:
            print("🚀 开始遥操作循环...")
            # print("按 Ctrl+C 停止")
            
            await teleop_loop_async(
                teleop=teleop,
                fps=cfg.fps,
                display_data=cfg.display_data,
                duration=cfg.teleop_time_s,
                sender=sender
            )
            
        except KeyboardInterrupt:
            print("\n⏹️ 用户中断操作")
        except Exception as e:
            logging.error(f"运行错误: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # 清理资源
            if cfg.display_data:
                rr.rerun_shutdown()
            teleop.disconnect()
            print("👋 遥操作结束，资源已清理。")

    # 运行异步主循环
    asyncio.run(run_async())


def main():
    """主函数"""
    teleoperate()


if __name__ == "__main__":
    main()