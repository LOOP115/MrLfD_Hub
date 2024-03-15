# Setup franka_ros2



## Install libfranka

```bash
sudo apt install -y libpoco-dev libeigen3-dev
mkdir project && cd project
git clone --recursive https://github.com/frankaemika/libfranka --branch 0.13.2
cd libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build . -j$(nproc)
cpack -G DEB

# Replace the * with the actual numbers in the file name, which can be found in the current directory (run "ls" to see).
sudo dpkg -i libfranka-*.deb
```



## Install franka_ros2 from source

1. Install requirements:

```bash
sudo apt install -y \
ros-humble-hardware-interface \
ros-humble-generate-parameter-library \
ros-humble-ros2-control-test-assets \
ros-humble-controller-manager \
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
ros-humble-ament-cmake \
ros-humble-ament-cmake-clang-format \
python3-colcon-common-extensions
```

2. Create a ROS 2 workspace:

```bash
cd
cd project && mkdir franka_ws
cd franka_ws && mkdir src && cd src
```

3. Clone repo and build packages:

```bash
git clone https://github.com/frankaemika/franka_ros2.git
cd franka_ros2 && git checkout humble
cd
cd project/franka_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```





## Troubleshooting

### Fail to build

`could not find a configuration file for package franka that is compatible with requested version 0.13.2`

- Remove uncompatible libfranka versions

```
dpkg -l | grep libfranka
sudo dpkg -r libfranka
sudo apt autoremove
```

