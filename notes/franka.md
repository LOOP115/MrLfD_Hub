```
sudo apt install -y libpoco-dev libeigen3-dev
mkdir project && cd project
git clone --recursive https://github.com/frankaemika/libfranka --branch 0.10.0
cd libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build . -j$(nproc)
cpack -G DEB

sudo dpkg -i libfranka-*.deb	#choose right version
```



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



```
cd project && mkdir franka_ws
cd franka_ws && mkdir src && cd src
git clone https://github.com/mcbed/franka_ros2.git
cd
cd franka_ros2 && git checkout humble
cd project/franka_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=/home/loop/project/libfranka/build
```



```
ros2 launch panda gz.launch.py

ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=dont-care use_fake_hardware:=true

ros2 launch franka_bringup franka.launch.py robot_ip:=dont-care use_fake_hardware:=true use_rviz:=true
```



```
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2

ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=172.16.0.2
```

