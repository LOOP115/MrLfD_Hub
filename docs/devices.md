## Devices

#### Meta Quest 3

- [Meta - Social features and sharing](https://www.meta.com/en-gb/help/quest/articles/in-vr-experiences/oculus-features/cast-with-quest/)
- [Meta Quest Developer Hub](https://developer.oculus.com/meta-quest-developer-hub/)



#### Intel RealSense Depth Camera D435if

* [Get Started](https://www.intelrealsense.com/get-started-depth-camera/)
* [Linux Distribution](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md)
* Start: `realsense-viewer`



#### Franka Emika Panda

[Franka_ROS2_Install](Franka_ROS2_Install.md)

[Notion](https://www.notion.so/chri-lab/Franka_ROS2_Install-c0a47bf95d0c42099bac79e859c46ac6)

**CLI examples**

**Simulation**

```bash
ros2 launch panda gz.launch.py

ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=dont-care use_fake_hardware:=true

ros2 launch franka_bringup franka.launch.py robot_ip:=dont-care use_fake_hardware:=true use_rviz:=true
```

**Real**

```bash
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2

ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=172.16.0.2
```

