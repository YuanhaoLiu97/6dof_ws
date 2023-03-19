# 安装vision-msgs 
sudo apt install ros-humble-vision-msgs
sudo apt install ros-humble-vision-msgs-layers
sudo apt install ros-humble-vision-msgs-rviz-plugins
sudo apt install ros-humble-message-filters

## 编译运行程序
colcon build --package-select super4pcs
ros2 run super4pcs super4pcs_ros2_interface --ros-args --params-file src/config/sixdof_config.yaml

ros2 run super4pcs detect3d_node --ros-args --params-file src/config/sixdof_config.yaml
