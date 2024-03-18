# Setup Franka



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
# Add these two lines to ~/.bashrc
source /opt/ros/humble/setup.bash
export RCUTILS_COLORIZED_OUTPUT=1
```



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

# Run the example
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

### Troubleshooting: Fail to build

`could not find a configuration file for package franka that is compatible with requested version 0.13.2`

- Make sure to install the right version of `libranka`

- Remove uncompatible libfranka versions

```
dpkg -l | grep libfranka
sudo dpkg -r libfranka
sudo apt autoremove
```



## Simulation

### Install requirements

```bash
sudo apt install -y \
ros-humble-ros-gz-sim \
ros-humble-moveit-servo \
ros-humble-ros-ign-bridge \
ros-humble-ros-ign
```

### Install Gazebo

```bash
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] [http://packages.osrfoundation.org/gazebo/ubuntu-stable](http://packages.osrfoundation.org/gazebo/ubuntu-stable) $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# note: the above operation should create a line in "/etc/apt/sources.list.d/gazebo-stable.list" and should look something like this (will vary according to computer architecture & the version of Ubuntu):
# deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main

sudo apt-get update
sudo apt-get install ignition-fortress
```

### Install the FR Panda packages for gazebo

```bash
# Create a workspace
cd
cd project && mkdir franka_sim
cd franka_sim && mkdir src && cd src

# Clone repo
git clone https://github.com/LOOP115/panda_ign_moveit2.git -b humble
```

### Install gz_ros2_control

```bash
# Might need this first
sudo apt-get install libignition-cmake2-dev

# Go to the src directory
cd
cd project/franka_sim/src

# Clone repo
git clone https://github.com/ros-controls/gz_ros2_control.git -b humble

gedit ~/.bashrc
# Add this line to ~/.bashrc
export IGNITION_VERSION=fortress

# Go back to workspace and build
cd ..
source ~/.bashrc
colcon build
```

### Launch Gazebo with Franka

```bash
# Add this line to .bashrc
source ~/project/franka_sim/install/setup.bash

ros2 launch panda gz.launch.py
```

### Troubleshooting: Segmentation fault on launch Gazebo

```bash
cd ~/project/franka_sim/src/gz_ros2_control/
git checkout 2c3c46fabfde600ca190c30d51288b8308e45d01
cd ../..
colcon build
```

