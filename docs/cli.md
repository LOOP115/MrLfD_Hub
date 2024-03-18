# CLI Memo



## General

```bash
# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"

# Launch Gazebo with Franka
ros2 launch panda gz.launch.py
```



## franka_ros2

```bash
# MoveIt
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2

# Dummy
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

# Move-to-start
ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=172.16.0.2

# Start the robot without any controllers
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2

# Load a controller
ros2 control load_controller --set-state active [controller]

# Stop a controller
ros2 control set_controller_state [controller] inactive
```

### **Example Controllers**

```bash
# Gravity Compensation
ros2 launch franka_bringup gravity_compensation_example_controller.launch.py robot_ip:=172.16.0.2

# Joint Impedance Example
ros2 launch franka_bringup joint_impedance_example_controller.launch.py robot_ip:=172.16.0.2

# Joint Impedance With IK Example
ros2 launch franka_bringup joint_impedance_with_ik_example_controller.launch.py robot_ip:=172.16.0.2

# Model Example Controller
ros2 launch franka_bringup model_example_controller.launch.py robot_ip:=172.16.0.2

# Joint Position Example
ros2 launch franka_bringup joint_position_example_controller.launch.py robot_ip:=172.16.0.2

# Joint Velocity Example
ros2 launch franka_bringup joint_velocity_example_controller.launch.py robot_ip:=172.16.0.2

# Cartesian Pose Example
ros2 launch franka_bringup cartesian_pose_example_controller.launch.py robot_ip:=172.16.0.2

# Cartesian Orientation Example
ros2 launch franka_bringup cartesian_orientation_example_controller.launch.py robot_ip:=172.16.0.2

# Cartesian Pose Elbow Example
ros2 launch franka_bringup cartesian_elbow_example_controller.launch.py robot_ip:=172.16.0.2

# Cartesian Velocity Example
ros2 launch franka_bringup cartesian_velocity_example_controller.launch.py robot_ip:=172.16.0.2

# Cartesian Elbow Example
ros2 launch franka_bringup elbow_example_controller.launch.py robot_ip:=172.16.0.2
```

