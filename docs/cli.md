# CLI Memo



## General

```bash
# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"

# Launch Gazebo with Franka
ros2 launch panda gz.launch.py
```



## ROS-Unity Integration Examples

```bash
# Get ROS machine's IP
hostname -I

# Go into workspace
source install/setup.bash

# Start the endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=10.61.144.175

# Publisher
ros2 topic echo pos_rot

# Subscriber
ros2 run unity_robotics_demo color_publisher

# Unity servive
ros2 service call obj_pose_srv unity_robotics_demo_msgs/ObjectPoseService "{object_name: Cube}"

# Service call
ros2 run unity_robotics_demo position_service
```



## pymoveit2

```bash
# Move to joint configuration
ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"

# Move to Cartesian pose (motion in either joint or Cartesian space)
ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False

# Repeatadly toggle the gripper (or use "open"/"close" actions)
ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="toggle"

# Example of using MoveIt 2 Servo to move the end-effector in a circular motion
ros2 run pymoveit2 ex_servo.py

# Example of adding a collision object with primitive geometry to the planning scene of MoveIt 2
ros2 run pymoveit2 ex_collision_primitive.py --ros-args -p shape:="sphere" -p position:="[0.5, 0.0, 0.5]" -p dimensions:="[0.04]"

# Example of adding a collision object with mesh geometry to the planning scene of MoveIt 2
ros2 run pymoveit2 ex_collision_mesh.py --ros-args -p action:="add" -p position:="[0.5, 0.0, 0.5]" -p quat_xyzw:="[0.0, 0.0, -0.707, 0.707]"
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

