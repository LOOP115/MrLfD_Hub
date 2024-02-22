## Configuration



#### Intel RealSense Depth Camera D435if

* [Get Started](https://www.intelrealsense.com/get-started-depth-camera/)
* [Linux Distribution](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md)



#### Franka

[Franka_ROS2_Install](Franka_ROS2_Install.md)

[Notion](https://www.notion.so/chri-lab/Franka_ROS2_Install-c0a47bf95d0c42099bac79e859c46ac6)

**Packages**

```
sudo apt install -y \
ros-humble-control-msgs \
ros-humble-xacro \
ros-humble-angles \
ros-humble-ros2-control \
ros-humble-realtime-tools \
ros-humble-control-toolbox \
ros-humble-moveit \
ros-humble-ros2-controllers \
ros-humble-joint-state-publisher \
ros-humble-joint-state-publisher-gui \
ros-humble-ament-cmake-clang-format \
ros-humble-ros-gz-sim \
ros-humble-moveit-servo \
ros-humble-ros-ign-bridge \
ros-humble-ros-ign
```

**Commands**

Virtual

```
ros2 launch panda gz.launch.py

ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=dont-care use_fake_hardware:=true

ros2 launch franka_bringup franka.launch.py robot_ip:=dont-care use_fake_hardware:=true use_rviz:=true
```

Real

```
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2

ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=172.16.0.2
```

