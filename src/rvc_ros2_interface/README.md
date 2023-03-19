# RVC ROS2 Interface
This repository contains the ROS2 humble interface for RVC Industrial 3D Camera.

## Installation

### Dependencies
pcl -> .12.1
opencv -> 4.5.4

If you have installed 'ros-galactic-desktop' successfully, run the following commands to install the dependencies.

```bash
sudo apt install libopencv-dev
sudo apt install ros-galactic-cv-bridge
sudo apt install libpcl-dev
sudo apt install ros-galactic-pcl-conversions
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-pcl*
```

### Install RVC API
Download RVC API from [RVC's website](rvbust.com/download.html) and install it with `sudo dkpg -i`.

### Problem occurred and solved
1. 编译时, 出现"WARNING ** io features related to pcap will be disables", 原因是PCL库链接问题,解决方法可参考 blog.csdn.net/u013834525/article/details/96843094

2. 编译时, 出现错误日志: ”cmake error while loading shared libraries no such file or directory | ldd => not found“ 主要原因是colcon跟cmake编译机制不太一样, 在引入第三方库的时候, Colcon没有添加RPath导致的.  具体可参考 blog.csdn.net/qq_28087491/article/details/128665295

3. 编译时，出现"pcl_conversions"错误
   可参考 "guyuehome.com/27938"

## Run
go to workspace, 
```bash
colcon build
source install/local_setup.bash
ros2 run rvc_ros2_interface start
#采集数据
ros2 service call /rvc/capture rvc_ros2_interface/srv/CaptureImagePc {}
#启动TF
ros2 run sixdof_dof2 tf_broadcaster
```

![](image/Screenshot%20from%202023-02-23%2017-37-41.png)

## 调试


## Others 
1. 点云与PointCloud2转换可参考
   * github.coom/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
   * https://mp.weixin.qq.com/s/_zp8pSJ9f766ZyATiiapVQ

2. ros2发布订阅image和PointCloud2
   blog.csdn.net/mengfanji_5995/article/details/127526564




