# CLI Memo



## **Dummy**

### Example Controllers

```bash
# MoveIt example
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

# Move-to-start
ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=dont-care use_fake_hardware:=true

ros2 launch franka_bringup franka.launch.py robot_ip:=dont-care use_fake_hardware:=true use_rviz:=true

ros2 launch panda gz.launch.py
```





## **Real**

```bash
# MoveIt example
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2

# Move-to-start
ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=172.16.0.2

# Gravity Compensation
ros2 launch franka_bringup gravity_compensation_example_controller.launch.py robot_ip:=172.16.0.2

# Joint Impedance Example
ros2 launch franka_bringup joint_impedance_example_controller.launch.py robot_ip:=172.16.0.2

# Joint Impedance With IK Example
ros2 launch franka_bringup joint_impedance_with_ik_example_controller.launch.py robot_ip:=172.16.0.2

# Model Example Controller
ros2 launch franka_bringup model_example_controller.launch.py robot_ip:=172.16.0.2

ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2
```

