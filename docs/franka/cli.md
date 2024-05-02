# CLI Memo

<br>

## General

```bash
# Launch Gazebo with Franka
ros2 launch panda gz.launch.py

# Covert XACRO to URDF
xacro panda_arm.urdf.xacro > panda.urdf
```

<br>

## franka_ros2

```bash
# MoveIt
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=<robot_ip>

# Dummy
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

# Move-to-start
ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=<robot_ip>

# Start the robot without any controllers
ros2 launch franka_bringup franka.launch.py robot_ip:=<robot_ip>

# Load a controller
ros2 control load_controller --set-state active [controller]

# Stop a controller
ros2 control set_controller_state [controller] inactive
```

<br>

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

