<img width="4277" height="589" alt="任务数据采集" src="https://github.com/user-attachments/assets/fc65cc36-5a97-4c8f-a260-ddb3b34b4d47" />


<div align="center">

# LEO-ROBOT

</div>

## 引言
本项目从零开始搭建lerobot实现特定任务场景的抓取。其中该项目以NVIDIA的GR00T N1.5作为基础VLA模型，将其微调并部署至lerobot SO-101ARM实机，本仓库介绍了整个实现流程以及部署时可能遇到的问题。

<!-- <img width="4277" height="589" alt="任务数据采集" src="https://github.com/user-attachments/assets/817a1bfd-30d4-46cc-8273-cf73088559d8" /> -->

## 简介
### 任务场景布置：桌面上摆放着多支笔和橡皮，SO101机械臂自行清理桌面: 先将笔和橡皮收拾进容器内，然后使用抹布擦拭桌面的长时序任务。
### 未来工作:形成一个便携,易于嵌入的低成本组件,可以接入:
- `底盘`:形成拖地机器人
- `无人机底座`:形成更高自由度的操作手平台
- `固定部署在家庭场景`:如儿童书桌
- 遥操平台准备接入基于 `Vision Pro` 等混合现实显示设备,使用姿态数据进行操作

## 数据采集：
**_后续会将数据集开源至 hugging face 社区。_**
我们采用多源数据集融合训练，数据集包含：
- `300组` 真机遥操作抓取
- `120组` 真机**随机域**遥操作抓取
- `80组` 特殊任务真机遥操
- `50组` Mujoco 仿真**随机域**遥操作抓取
- `50组` Isaac Sim 仿真遥操作抓取

https://github.com/user-attachments/assets/c786341e-9a82-4005-9905-f5d889dbb5be

## 项目效果：
### 1）开环测试
- 平均`MSE`达到`7.171`(越小越好)
- 最小`MSE`达到`4.536`(越小越好)
- 作为对比,现有已发布的同类型任务[Post-Training Isaac GR00T N1.5 for LeRobot SO-101 Arm](https://huggingface.co/blog/nvidia/gr00t-n1-5-so101-tuning#post-training-isaac-gr00t-n15-for-lerobot-so-101-arm)在相同验证集中测得的`MSE`为`10.416`,误差较大;且相对任务长度短,不具备长任务能力

<img width="1814" height="1125" alt="image" src="https://github.com/user-attachments/assets/ad490c3c-e180-4cd7-b8ec-64ef4e5e79ff" />

对轨迹进行评估（取6条示例）

### 2）真机推理效果
我们在不同`环境/光线/任务场景`下进行了多次验证,证明本项目已达到**稳定,平滑,迅捷,精确**的指标

https://github.com/user-attachments/assets/08f7c784-6e52-46de-a75d-054de376decd

黑客松比赛现场环境推理:

https://github.com/user-attachments/assets/c7de2c26-3cc9-4c30-a83d-2f3bfa16a6dd


----
## 安装指南
### 1）根据参考地址1创建lerobot虚拟环境和克隆lerobot仓库以便SO101的安装与使用
```
git clone https://gitee.com/Marlboro1998/lerobot.git ~/lerobot
conda create -y -n lerobot python=3.10 && conda activate lerobot
```
根据教程安装相关依赖和环境搭建
### 2）根据参考地址2创建GR00T虚拟环境和克隆GR00T仓库以便模型部署与使用
#### 注意：这里需要用到两个GR00T的环境，因为后续部署到lerobot上时需要在其中一个GR00T环境中安装lerobot相关依赖，这会对原GR00T环境有影响。
```
git clone https://github.com/NVIDIA/Isaac-GR00T
cd Isaac-GR00T
conda create -n gr00t-server python=3.10
conda activate gr00t-server
pip install --upgrade setuptools
pip install -e .[base]
pip install --no-build-isolation flash-attn==2.7.1.post4
```
在同一个目录下安装另一个GR00T环境（或直接复制一个不同名的虚拟环境），并在此之后安装lerobot依赖以便后续使用
```
conda create -n gr00t-client python=3.10
conda activate gr00t-client
pip install --upgrade setuptools
pip install -e .[base]
pip install --no-build-isolation flash-attn==2.7.1.post4
```
在gr00t-client安装以下依赖，主要为了后续运行带有lerobot相关内容的推理脚本

`cd [your path of lerobot] && pip install -e ".[feetech]" # 进入你lerobot目录（一般是.../lerobot/src/lerobot）下安装对应依赖`

## 3实现过程
### 3.1 数据采集
先查看和确保机械臂与相机的连接是否正常以及查看对应的端口，可在终端输入以下指令：
```
conda activate lerobot # 需要先激活lerobot环境
lerobot-find-cameras opencv # 查看摄像头的接入信息
```
<img width="600" height="252" alt="2025-10-15 14-33-03 的屏幕截图" src="https://github.com/user-attachments/assets/522f2799-8589-4d62-95b6-35d4514d5b07" />

其中“Id: /dev/video0”信息中video后面的数字即为该摄像头的编号，后面需要用到！
```
ls /dev/ttyACM* # 查看机械臂的接入端口
```
<img width="706" height="33" alt="2025-10-15 14-38-18 的屏幕截图" src="https://github.com/user-attachments/assets/c6bc9f88-d5d4-4e3e-967a-82c362e5fda5" />

可通过插拔确定具体主从对应的端口，注意不要搞反！
```
sudo chmod 666 /dev/ttyACM* # 赋予端口权限
```

然后调用采集数据的脚本收集数据，若之前未进行校准会先提示进行校准，校准文件会保存在~/.cache/huggingface/lerobot/calibration路径下，若想重新校准需先删除改路径下存在的校准文件
```
lerobot-record \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=my_awesome_follower_arm \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}" \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=my_awesome_leader_arm \
    --display_data=true \
    --dataset.repo_id=seeedstudio123/pen_and_cloth \
    --dataset.num_episodes=5 \
    --dataset.single_task="place the pens and eraser into the white bowl,then use the cloth to clean the table,finally place the cloth next to the white bowl" \
    --dataset.push_to_hub=false \
    --dataset.episode_time_s=40 \
    --dataset.reset_time_s=5 
```
其中repo_id可以自定义修改，push_to_hub=false，最后数据集会保存在主目录的~/.cache/huggingface/lerobot下会创建上述seeedstudio123/test文件夹，如果记录过程中断，可以通过重新运行相同的命令并添加 --resume=true 来恢复记录
#### 注意：lerobot原仓库内的代码对数据集的保存做了文件大小的判断，一般会把所有episode整合保存在一个文件里，所以为了后续方便数据转换，建议对lerobot/src/lerobot/dataset/utils.py文件进行以下修改！！！
<img width="564" height="75" alt="2025-10-15 15-07-45 的屏幕截图" src="https://github.com/user-attachments/assets/bda8dc5e-5c85-4cfd-8be1-bffced0e6c60" />

### 3.2 数据转换
#### GR00T需要将原始的lerobot数据文件添加相关配置文件才可训练（具体见GR00T官网介绍），因此我们自行编写了一个dataset_le2gr00t.py来进行数据转换操作,该代码存放在/ISSAC-GR00T-LE/scripts/文件夹下
先将录制的数据集移动到/ISSAC-GR00T-LE/demo_data目录下，然后终端cd到/ISSAC-GR00T-LE目录下运行
```
conda activate gr00t-server # 激活gr00t-server虚拟环境
python scripts/dataset_le2gr00t.py # 运行转换脚本，根据提示选择数据集  
```
<img width="1132" height="321" alt="2025-10-15 15-35-06 的屏幕截图" src="https://github.com/user-attachments/assets/b3da6931-1404-42b9-a5c6-16ce6ede8ede" />

#### 注意：这里脚本会创建一份modality.json的文件，跟摄像头配置有关系（具体见GR00T官网介绍），若在录制时改变了摄像头配置，需在该数据转换脚本里修改相应的内容。建议如果使用两个摄像头的配置，不要修改"front"和“wrist”字段，只需自行清楚哪个对应哪个index_or_path即可。

### 3.3 微调训练
在gr00t-server环境下运行：
```
python scripts/gr00t_finetune.py \
   --dataset-path ./demo_data/pen_and_cloth/ \
   --num-gpus 1 \
   --output-dir ./finetuned_models/pen_and_cloth  \
   --max-steps 20000 \
   --data-config so100_dualcam \
   --video-backend torchvision_av
```
若GPU显存较小，冻结diffusion微调,或减小--batch_size 16
```
python scripts/gr00t_finetune.py \
   --dataset-path ./demo_data/pen_and_cloth/ \
   --num-gpus 1 \
   --output-dir ./finetuned_models/pen_and_cloth \
   --max-steps 20000 \
   --data-config so100_dualcam \
   --video-backend torchvision_av \
   --no-tune_diffusion_model
```
### 3.4 开环评估
在gr00t-server环境下运行：
```
python scripts/eval_policy.py --plot \
   --embodiment_tag new_embodiment \
   --model_path ./finetuned_models/pen_and_cloth_df \
   --data_config so100_dualcam \
   --dataset_path ./demo_data/pen_and_cloth/ \
   --video_backend torchvision_av \
   --modality_keys single_arm gripper \
   --trajs=10
```
<img width="350" height="400" alt="2025-10-15 16-17-57 的屏幕截图" src="https://github.com/user-attachments/assets/ff112f42-5ffb-4f98-a00f-1e4c548f8511" />

### 3.5 真机部署
在gr00t-server环境下运行：
```
python scripts/inference_service.py --server \
    --model_path ./finetuned_models/pen_and_cloth_df \
    --embodiment-tag new_embodiment \
    --data-config so100_dualcam \
    --denoising-steps 4
```
然后新开一个终端，在gr00t-client环境下运行：
```
python examples/SO-100/eval_lerobot.py \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=my_awesome_follower_arm \
    --robot.cameras="{ wrist: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}, front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
    --policy_host=127.0.0.1 \
    --lang_instruction="place the pens and eraser into the white bowl,then use the cloth to clean the table,finally place the cloth next to the white bowl."
```

#### 实现过程主要参考以下地址:
[1、lerobot安装使用教程](https://wiki.seeedstudio.com/cn/lerobot_so100m_new/)

[2、GR00TN1.5仓库](https://github.com/NVIDIA/Isaac-GR00T)




