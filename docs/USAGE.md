
# 📖使用教程
[![README](https://img.shields.io/badge/_返回README-文档-blue?style=flat&logo=readme)](../README.md)

## 1. 数据采集
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

## 2. 数据转换
#### GR00T需要将原始的lerobot数据文件添加相关配置文件才可训练（具体见GR00T官网介绍），因此我们自行编写了一个dataset_le2gr00t.py来进行数据转换操作,该代码存放在/ISSAC-GR00T-LE/scripts/文件夹下
先将录制的数据集移动到/ISSAC-GR00T-LE/demo_data目录下，然后终端cd到/ISSAC-GR00T-LE目录下运行
```
conda activate gr00t-server # 激活gr00t-server虚拟环境
python scripts/dataset_le2gr00t.py # 运行转换脚本，根据提示选择数据集  
```
<img width="1132" height="321" alt="2025-10-15 15-35-06 的屏幕截图" src="https://github.com/user-attachments/assets/b3da6931-1404-42b9-a5c6-16ce6ede8ede" />

#### 注意：这里脚本会创建一份modality.json的文件，跟摄像头配置有关系（具体见GR00T官网介绍），若在录制时改变了摄像头配置，需在该数据转换脚本里修改相应的内容。建议如果使用两个摄像头的配置，不要修改"front"和“wrist”字段，只需自行清楚哪个对应哪个index_or_path即可。

## 3. 微调训练
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
## 4. 开环评估
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

## 5. 真机部署
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

