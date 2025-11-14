<img width="4277" height="589" alt="任务数据采集" src="https://github.com/user-attachments/assets/fc65cc36-5a97-4c8f-a260-ddb3b34b4d47" />


<div align="center">
    


<img src="https://github.com/user-attachments/assets/e10c5606-ecd3-48fb-96a5-66c8ee38a289" height="200" alt="hackathon_final">
<img src="https://github.com/user-attachments/assets/284fc117-0967-4f30-8a87-a1e58db630d3" height="200" alt="mj_50_crop">
<img src="https://github.com/user-attachments/assets/87e68994-25bf-4d90-bc51-655ea9dd4567" height="200" alt="300组+滤波+限速_剪切">

# LEO-ROBOT

[![GitHub](https://img.shields.io/badge/仓库-GitHub-181717?style=flat&logo=github)](https://github.com/Lee-LEO-H/Lerobot-GR00TN1.5)
[![Install](https://img.shields.io/badge/🔧_安装-Guide-blue?style=flat)](./docs/INSTALL.md)
[![Usage](https://img.shields.io/badge/📖_用法-Guide-green?style=flat)](./docs/USAGE.md)
[![Demo](https://img.shields.io/badge/_效果-Video-red?style=flat&logo=youtube)](./media)
<!-- [![Model](https://img.shields.io/badge/_模型-HuggingFace-yellow?style=flat&logo=huggingface)](https://huggingface.co/your-model) -->

</div>



## 引言
本项目搭建基于lerobot实现特定任务场景的抓取。以NVIDIA的GR00T N1.5作为基础VLA模型，微调并部署至lerobot SO-101ARM实机，本文介绍了项目效果、实现流程以及部署时可能遇到的问题。

<!-- <img width="4277" height="589" alt="任务数据采集" src="https://github.com/user-attachments/assets/817a1bfd-30d4-46cc-8273-cf73088559d8" /> -->

## 简介
任务场景布置：桌面上摆放着多支笔和橡皮，SO101机械臂自行清理桌面: 先将笔和橡皮收拾进容器内，然后使用抹布擦拭桌面的长时序任务。

## 数据采集：
我们采用多源数据集共300个episode融合训练，数据集包含：约70%实验室真机遥操作数据，20%现场真机遥操作，10%仿真数据，采集过程均加入域随机化（即笔的位置、类型、形态以及篮子的位置等均随机摆放）

https://github.com/user-attachments/assets/c786341e-9a82-4005-9905-f5d889dbb5be

## 项目效果：
### 1）开环测试
- 平均`MSE↓`达到`7.171`(越小越好)
- 最小`MSE↓`达到`4.536`(越小越好)
- 作为对比,现有已发布的同类型任务[Post-Training Isaac GR00T N1.5 for LeRobot SO-101 Arm](https://huggingface.co/blog/nvidia/gr00t-n1-5-so101-tuning#post-training-isaac-gr00t-n15-for-lerobot-so-101-arm)在相同验证集中测得的`MSE`为`10.416`,误差较大;且相对而言任务长度短,不具备长任务能力

<img width="1814" height="1125" alt="image" src="https://github.com/user-attachments/assets/ad490c3c-e180-4cd7-b8ec-64ef4e5e79ff" />

对轨迹进行评估（取6条示例）

### 2）真机推理效果
我们在不同`环境/光线/任务场景`下进行了多次验证,展示了本项目已达到相对**稳定,平滑,迅捷,精确**的指标

- 实验室环境推理：`4090`(speed X1.0）

https://github.com/user-attachments/assets/08f7c784-6e52-46de-a75d-054de376decd

- 黑客松比赛现场环境推理: `Jetson Thor`(speed X1.0）

https://github.com/user-attachments/assets/c7de2c26-3cc9-4c30-a83d-2f3bfa16a6dd

- MuJoCo仿真环境推理：`4090`

https://github.com/user-attachments/assets/26e6f3a1-8160-4c02-ada8-2bfa60678c4f

- 展现出一定的泛化效果：`Jetson Thor`

https://github.com/user-attachments/assets/a831f46b-edef-4787-a5d3-5251c5ec3f4a

## 未来工作:
形成一个便携,易于嵌入的低成本组件,可以接入:
- `底盘`:形成拖地机器人
- `无人机底座`:形成更高自由度的操作手平台
- `固定部署在家庭场景`:如儿童书桌
- 遥操平台准备接入基于 `Vision Pro` 等混合现实显示设备,使用姿态数据进行操作

### 具体实现过程可查阅docs目录下的教程 ^_^

----



