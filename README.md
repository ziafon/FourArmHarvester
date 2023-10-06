# 四臂采摘机器人控制系统

## 项目简介
这是适用于北京农科院装备中心的一款四臂采摘机器人的控制系统，它可以感知果实位置，规划机器人采摘次序并控制执行器自动完成采摘作业。对于此系统有几点说明：
* 此项目目前只适用于北京农科院装备中心的一款四臂采摘机器人；
* 此项目配置需要11代酷睿i5及以上的处理器，16GB运行内存，建议使用11代处理器以获得更好的兼容性和稳定性；
* 此项目目前只能运行于Ubuntu20.04LTS系统上，其他系统或版本暂不支持。
## 项目部署
默认用户已有一台符合上述要求的电脑主机
### 安装系统
1、具体参考[Ubuntu 20.04 LTS 桌面版详细安装指南](https://www.sysgeek.cn/install-ubuntu-20-04-lts-desktop/)进行操作系统安装，操作系统可以从开源镜像站下载：
```
https://mirrors.huaweicloud.com 华为开源
https://mirrors.tuna.tsinghua.edu.cn 清华开源下载
https://mirrors.ustc.edu.cn 中国科学技术大学开源软件镜像
```
### 安装ROS
ROS是专为机器人开发的高效软件框架，此项目也是基于ROS进行设计，安装ROS步骤如下。
* 切换国内源以提高安装速度：
```
sudo sh -c '. /etc/lsb-release && echo "d{2d3b390d-f615-41d0-a830-7bbdcedbd397}eb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```
```
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```
* 安装KEY
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
* 更新源
```
sudo apt update
```
* 安装ROS
```
sudo apt install ros-noetic-desktop 
```
* 配置环境变量
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
```
source ~/.bashrc
```
### 安装其他工具包
一些ROS关联的工具包并不在上述安装的软件集中，需要单独安装。
```
sudo apt install python3-opencv
sudo apt install ros-noetic-librealsense2
sudo apt install ros-noetic-realsense2-camera
sudo apt install ros-noetic-realsense2-description
```
上述命令安装视觉处理库OpenCV以及深度相机的驱动。
### 安装Python功能库
本项目采用Python语言开发，依赖于Python相关的第三方库，若要安装这些库，首先安装Python包管理工具pip
```
sudo apt install python3-pip
```
然后一键安装相关三方库。工程目录下的requirements.txt文件已经包含所有需要的Python三方库，使用"pip3 install -r requirements"可以直接安装所有需要的库。另外使用国内镜像源，以加快下载速度。
```
pip3 install -r requirements -i https://pypi.tuna.tsinghua.edu.cn/simple
```
其中"-i https://pypi.tuna.tsinghua.edu.cn/simple"表示使用国内清华镜像。
### 修改配置文件
Ubuntu系统驱动外置USB设备需要使用权限，若要直接驱动外置CAN控制器，将本工程目录下的99-usb拷贝到如下目录。
```
sudo cp 99-myusb.rules /etc/udev/rules.d/
```
另外本机器人采用的CAN控制器需要对can库做一下修改，现有修改好的文件，复制到CAN库目录下覆盖原文件。
```
cp canalystii /home/arl/.local/lib/python3.8/site-packages/can/interfence/
```
## 项目使用
* 确认硬件已连接，系统已上电。
* 启动终端，然后运行以下命令初始化ROS，初始化机器人模型以及硬件：
```
roslaunch start.launch
```
随后在工程目录下新建终端运行如下命令启动主程序以及其他功能节点：
```
python3 main.py
```
这时项目的所有功能都已经启动，同时会生成一个简易的UI界面，通过界面可以尝试机器人的相关功能，比如变更机械臂位置，规划采摘目标以及执行采摘等。
## 其他说明
此项目目前为测试版本，且一些功能代码目前还在作大的改动，目前没有提交，后续会逐步完善。
## 版本更新日志
* v0.1 (2023.6.14)\
项目创建，带有基础功能。
* v0.2 (2023.7.9)\
实现感知、规划、控制以及驱动等功能。
* v0.2 (2023.7.20)\
优化驱动节点，删除无效的状态信息反馈。
* v0.3 (2023.8.6)\
新增统一配置文件，减少功能节点中的硬编码。
* v0.4 (2023.8.16)\
采摘控制部分重写，将之前的调度部分由轮询的方式改为线程并行的方式，提高稳定性。
* v0.41 (2023.8.29)\
修复新控制部分bug
* v0.5 (2023.9.11)\
规划部分和感知部分也重写
* v0.6 (2023.9.18)\
修复软件bug
* v0.7 (2023.9.24)
功能进一步集成
* v0.71 (2023.10.5)
功能进一步集成