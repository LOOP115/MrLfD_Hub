# Setup Gazebo with Franka

<br>

## Install requirements

```bash
sudo apt install -y \
ros-humble-ros-gz-sim \
ros-humble-moveit-servo \
ros-humble-ros-ign-bridge \
ros-humble-ros-ign
```

<br>

## Install Gazebo

```bash
sudo apt update
sudo apt install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] [http://packages.osrfoundation.org/gazebo/ubuntu-stable](http://packages.osrfoundation.org/gazebo/ubuntu-stable) $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# note: the above operation should create a line in "/etc/apt/sources.list.d/gazebo-stable.list" and should look something like this (will vary according to computer architecture & the version of Ubuntu):
# deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main

sudo apt update
sudo apt install ignition-fortress
```

<br>

## Install the FR Panda packages for gazebo

```bash
# Create a workspace
cd
cd project && mkdir franka_sim
cd franka_sim && mkdir src && cd src
```

```bash
# Clone repo
git clone https://github.com/LOOP115/panda_ign_moveit2.git -b humble
```

<br>

## Install gz_ros2_control

```bash
# Might need this first
sudo apt install libignition-cmake2-dev
```

```bash
# Go to the src directory
cd
cd project/franka_sim/src

# Clone repo
git clone https://github.com/ros-controls/gz_ros2_control.git -b humble
```

```bash
gedit ~/.bashrc
# Add this line to ~/.bashrc
export IGNITION_VERSION=fortress
```

```bash
# Go back to workspace and build
cd ..
source ~/.bashrc
colcon build --merge-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

```bash
# Symbolic link install (optional)
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

<br>

## Launch Gazebo with Franka

```bash
# Add this line to .bashrc
source ~/project/franka_sim/install/setup.bash
```

```bash
ros2 launch panda gz.launch.py
```

### Troubleshooting: Segmentation fault on launch Gazebo

```bash
cd ~/project/franka_sim/src/gz_ros2_control/
git checkout 2c3c46fabfde600ca190c30d51288b8308e45d01
cd ../..
colcon build --merge-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

<br>

## Install pymoveit2

```bash
# Go to the src directory
cd
cd project/franka_sim/src
```

```bash
# Clone and build
git clone https://github.com/LOOP115/pymoveit2
cd ..
colcon build --merge-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

