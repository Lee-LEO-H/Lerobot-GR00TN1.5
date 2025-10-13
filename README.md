# Lerobot-sim-real
## 0引言
### 本项目从零开始搭建lerobot实现特定任务场景的抓取。其中该项目以NVIDIA的GR00T N1.5作为基础VLA模型，将其微调部署lerobot SO-101ARM实机，本仓库介绍了整个实现流程以及部署时遇到的问题。
#### 实现过程主要参考以下地址
[1、lerobot安装使用教程](https://wiki.seeedstudio.com/cn/lerobot_so100m_new/)

[2、GR00TN1.5仓库](https://github.com/NVIDIA/Isaac-GR00T)

## 1简介
### 任务场景布置：在干净的桌面上摆放着多支笔和橡皮，操控SO101机械臂实现先将笔和橡皮收拾进容器内，然后使用抹布擦拭桌面的长时序任务。

### 项目效果：


## 2安装指南
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




![图片插入](https://www.baidu.com/img/bd_logo1.png) 
