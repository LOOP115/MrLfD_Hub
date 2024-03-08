# LfD on Franka



## Control

### Status lights

- Red: reach joint limits
- Yellow: singular configuration



### Torque Direction

- counter clockwise `+`

- clockwise `-`





## Tasks

* Detect object's location and pose
  * Depth Camera
  * Tag
* Collect trajectory data
* Save as hdf5
* Training

* Run the policy





## ROS TCP Connector

### CLI

```
hostname -I

colcon build
source install/setup.bash

ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=10.100.238.188
```

