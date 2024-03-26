# Setup franka_ros2

<br>

## Preparations

- Install [Ubuntu 22.04 LTS (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)
- Install Python

```bash
# Might need these dependencies
pip install catkin_pkg empy lark pytest jinja2 pyaml typeguard
```

- Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

```bash
gedit ~/.bashrc
```

```bash
# Add these two lines to ~/.bashrc
source /opt/ros/humble/setup.bash
export RCUTILS_COLORIZED_OUTPUT=1
```

<br>

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
```

```bash
# Replace the * with the actual numbers in the file name, which can be found in the current directory (run "ls" to see).
sudo dpkg -i libfranka-*.deb
```

<br>

## Install franka_ros2

### Install requirements

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

### Build packages

```bash
# Create a workspace
cd
cd project && mkdir franka_ws
cd franka_ws && mkdir src && cd src
```

```bash
# Clone and build
git clone https://github.com/frankaemika/franka_ros2.git
cd franka_ros2 && git checkout humble
cd
cd project/franka_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Verify the installation

```bash
# Add this line to ~/.bashrc
source ~/project/franka_ws/install/setup.bash
```

```bash
# Run the example
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

### Troubleshooting: Fail to build

`could not find a configuration file for package franka that is compatible with requested version 0.13.2`

- Make sure to install the right version of `libranka`

- Remove uncompatible libfranka versions

```bash
dpkg -l | grep libfranka
sudo dpkg -r libfranka
sudo apt autoremove
```

