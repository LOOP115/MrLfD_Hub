# Franka_ROS2_Install

Authors: @Muhammad Bilal @Sarah Schombs @Jiahe Pan 
Step-by-Step Guide to Install Resources for Franka Emika Panda Robot



## Ubuntu

Download and install Ubuntu 22.04.2 LTS (Jammy Jellyfish)

- Go to this link
    - [https://releases.ubuntu.com/jammy/](https://releases.ubuntu.com/jammy/)
    - and download the “Desktop Image”



## ROS2

Install ROS2 (Humble Hawksbill)

- Follow the instructions in the official documentation (ROS Humble)
    - [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)



## Libfranka

Follow the instruction here: 

Install libfranka by executing the following commands in terminal:

```bash
sudo apt install -y libpoco-dev libeigen3-dev
mkdir project && cd project
# Note: the following version "0.10.0" is required to be able to successfully connect to the "Franka Research 3 arm" (different to the older "Panda" arm)
git clone --recursive https://github.com/frankaemika/libfranka --branch 0.10.0
cd libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build . -j$(nproc)
cpack -G DEB
sudo dpkg -i libfranka-*.deb    # replace the * with the actual numbers in the file name, which can be found in the current directory (run "ls" to see)
```

go back to home directory with “cd” command

→ The official documentation for this part can be found here:

- [https://frankaemika.github.io/docs/installation_linux.html](https://frankaemika.github.io/docs/installation_linux.html)
- ******************IMPORTANT******************
    - This results in new “shared library files” (.so) files for libfranka, which can now be found and verified at “/usr/lib/”
    - We need to then re-build franka_ws (which contains the package franka_ros2)
    - to do this, simply remove the build, install and log folders in franka_ws, and then colcon build



## Franka_ros2

For franka_ros2, install the required packages by executing the following commands
in terminal:

```bash
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
python3-colcon-common-extensions
```

Open the bashrc file by entering the following command:

```bash
sudo vim ~/.bashrc
```

In `.bashrc` file, add the given line in the end of file and then save & close the file:

```bash
source /opt/ros/humble/setup.bash
export RCUTILS_COLORIZED_OUTPUT=1
```

close the file using esc, then :wq

or use `gedit ~/.bashrc`



## Workspace

Create a workspace for ros2 and clone the given github repository:

- go back to home directory, using cd

```bash
cd project && mkdir franka_ws
cd franka_ws && mkdir src && cd src
git clone [https://github.com/mcbed/franka_ros2.git](https://github.com/mcbed/franka_ros2.git)
cd franka_ros2 && git checkout humble
cd ..
cd project/franka_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=/home/<path_to_your_libfranka_folder>/build
```



## Source the built project

Open the bashrc file and add the following lines

```bash
source /home/<your_usernmae>/project/franka_ws/install/setup.sh
```



## Run the example

The above steps are also mentioned here (except github repository). Furthermore,
execute the example to verify the installation.

```bash
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```



## Install Gazebo

Install Gazebo Fortress by executing the following commands:

```bash
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget [https://packages.osrfoundation.org/gazebo.gpg](https://packages.osrfoundation.org/gazebo.gpg) -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] [http://packages.osrfoundation.org/gazebo/ubuntu-stable](http://packages.osrfoundation.org/gazebo/ubuntu-stable) $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# note: the above operation should create a line in "/etc/apt/sources.list.d/gazebo-stable.list" and should look something like this (will vary according to computer architecture & the version of Ubuntu):
# deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main

sudo apt-get update
sudo apt-get install ignition-fortress
```



## Install the FR Panda packages for gazebo

Install the following package for panda model in gazebo:

```bash
cd project/franka_ws/src
git clone [https://github.com/AndrejOrsula/panda_ign_moveit2.git](https://github.com/AndrejOrsula/panda_ign_moveit2.git)
cd panda_ign_moveit2
git checkout humble
cd && cd project/franka_ws
```

## Important: Extra steps needed!

Before **building**, we need to edit 2 bash scripts which generate **sdf** and **urdf** files from **xacro** files of the panda arm. 

- go into /franka_ws/src/panda_ign_moveit2/panda_description/scripts/
- open “xacro2sdf.bash”
- look at **line 16**, change “**effort**” to “**position**”
- do the **exact same** for “xacro2urdf.bash”

Now, we can simply go to the ********************************root of the workspace******************************** and **********build**********:

```cpp
cd && cd project/franka_ws
colcon build
```

## Old Debug Steps (“100% will work” - Muhammad)

- (If the above didn’t work), Navigate to the model.sdf file:

```bash
gedit ~/franka_ws/src/panda_ign_moveit2/panda_description/panda/model.sdf
```

- Look at 730/731 line and
  
    change “*`controllers_effort.yaml`*” to “*`controllers_position.yaml`*”
    
- Create two folders at the following location:
    - `cd /home/project/franka_ws/install/panda_description/lib`
    - `mkdir panda && mkdir urdf`
    - copy `model.sdf` file from `src/panda_ign_moveit2/panda_description/panda` folder and add `model.sdf` file to the panda folder you just created at
    `install/panda_description/lib`. Do the same with the `panda.urdf.xacro`;
    copy the file from src folder and paste it to the urdf folder you just
    created
    - go back to the `franka_ws` and build the workspace again with
    command “`colcon build`”
    



## Install gz_ros2_control (required to launch gazebo)

1. Go to the src directory of your workspace
    1. (might need this first) → “`sudo apt-get install libignition-cmake2-dev`” (link to the discussion on the issue: [https://github.com/PX4/PX4-Autopilot/issues/20923](https://github.com/PX4/PX4-Autopilot/issues/20923))
    2. `git clone [https://github.com/ros-controls/gz_ros2_control.git](https://github.com/ros-controls/gz_ros2_control.git) -b humble`
    3. `cd gz_ros2_control/`
    4. `git status`
    5. go to your ros2 workspace using `cd ../..`
    6. `gedit ~/.bashrc`
    7. add “`export IGNITION_VERSION=fortress`” in the bottom of the file
    8. go back to your workspace
    9. `source ~/.bashrc`
    10. `colcon build`



## Launching Gazebo with the Panda Arm

- (make sure the environment of “franka_ws” is sourced, or even better the line is added to `.bashrc` so don’t have to remember every time)
- `ros2 launch panda gz.launch.py`



## Installing the Bridge

- `sudo apt-get install ros-humble-ros-ign-bridge`
- `sudo apt-get install ros-humble-ros-ign`



## Defining robot collision behavior (safety)

- Go into /franka_ros2/franka_hardware/src and open “robot.cpp”
- Add the following function after “initializeContinuousReading()” (the actual threshold values should be changed according to your personal preferences → but don’t go too crazy 😎)

```cpp
void Robot::setForceTorqueCollisionBehavior()
{ 
  for (int i=0; i<10; i++) std::cout << "==========================================================" << std::endl;
  std::cout << "Setting collision behavior" << std::endl;
  for (int i=0; i<10; i++) std::cout << "==========================================================" << std::endl;

  std::array<double, 7> lower_torque_thresholds_nominal {10, 10, 10, 10, 10, 10, 10};
  std::array<double, 7> upper_torque_thresholds_nominal {15, 15, 15, 15, 15, 15, 15};

  std::array<double, 6> lower_force_thresholds_nominal {10, 10, 10, 10, 10, 10};
  std::array<double, 6> upper_force_thresholds_nominal {15, 15, 15, 15, 15, 15};

  robot_->setCollisionBehavior(lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                             lower_force_thresholds_nominal, upper_force_thresholds_nominal);

  for (int i=0; i<10; i++) std::cout << "==========================================================" << std::endl;
  std::cout << "Finished setting the requested collision behavior of the robot!" << std::endl;
  for (int i=0; i<10; i++) std::cout << "==========================================================" << std::endl;
}
```

- Open “franka_hardware_interface.cpp”, add a line inside the “on_activate” function, after “robot_→initializeContinuousReading()”
    - The function should look like this afterwards

```cpp
CallbackReturn FrankaHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_->initializeContinuousReading();

  robot_->setForceTorqueCollisionBehavior();   // this is the added line

  hw_commands_.fill(0);
  read(rclcpp::Time(0),
       rclcpp::Duration(0, 0));  // makes sure that the robot state is properly initialized.
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}
```

- Go back to the root of /franka_hardware, and then go into /include/franka_hardware, and then add the following line to “robot.hpp” (after “void initializeContinuousReading()”).

```cpp
void setForceTorqueCollisionBehavior();  // this is the added line
```

- Go back to the ********root******** of the franka_ws
- **colcon build**
- Double check that the build produced an updated **robot.hpp** file in the install folder of the workspace
    - cd **/install/franka_hardware/include/franka_hardware**
    - open up **robot.hpp**
    - make sure that “**void setForceTorqueCollisionBehavior()**” is there
    - **IF IT IS NOT THERE**
        - can simply remove the “**build**”, “**install**” and “**log**” folders from the root of **franka_ws**
            - [in the ********root******** of **franka_ws**] rm -r build install log
            - and then **colcon build** (in the root of franka_ws)



## Properly enable real-time kernel on Ubuntu 22.04 (solves “communication_constraint_violation”)

- Create a new Ubuntu One account using your email
- Find the free token and copy it
- To attach your personal machine to a free Pro subscription, run: `sudo pro attach <your_token>`.
- To enable the real-time beta kernel, run:

```bash
sudo pro enable realtime-kernel --b
```

- Restart the computer, and verify that real-time kernel is running
    - run: `uname -r`.
- After the `PREEMPT_RT` **kernel** is running, add a group named **realtime** and add the user controlling your robot to this group:

```bash
sudo addgroup **realtime**
sudo usermod -a -G realtime $(whoami)
```

- Afterwards, add the following limits to the **realtime** group in `/etc/security/limits.conf`:

```
@**realtime** soft rtprio 99
@**realtime** soft priority 99
@**realtime** soft memlock 102400
@**realtime** hard rtprio 99
@**realtime** hard priority 99
@**realtime** hard memlock 102400
```

- **IMPORTANT:**
    - The limits will be applied after you **log out and in** again. (if not, restart the computer)



## Enable “performance” mode of all CPUs

- `sudo apt install cpufrequtils`
- `cpufreq-info`
- Then, execute the following commands

```
sudo systemctl disable ondemand
sudo systemctl enable cpufrequtils
sudo sh -c 'echo "GOVERNOR=performance" > /etc/default/cpufrequtils'
sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils
```



## Launching controllers through LAUNCH FILES

- package “controller_manager” is located at /opt/ros/humble/lib/controller_manager/
- there are 3 **********************executable********************** in this package here - {ros2_control_node, spawner, unspawner}
- go into this directory, and check how to use the **spawner** - “./spawner -h” (for help)
    - We can see that in the “**franka.launch.py**” launch file, we first launch the **ros2_control_node**, with one of the parameters passed in being the **franka_controllers**
    - The **franka_controllers** are defined in **/franka_bringup/config/controllers.yaml**, however sadly there are only 5 controllers in there originally (people are lazy).
    - Firstly, change the type (class) of the 4th controller so that it looks like this:

```yaml
		gravity_compensation_example_controller:
      type: franka_example_controllers/GravityCompensationExampleController

    joint_impedance_example_controller:
      type: franka_example_controllers/JointImpedanceExampleController

    move_to_start_example_controller:
      type: franka_example_controllers/MoveToStartExampleController

    joint_trajectory_controller:   # this is where we change the controller
      # type: joint_effort_trajectory_controller/JointTrajectoryController
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```

- Now, we can simply add a node to our launch file as such:

```python
	  Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'], # or any other controller
        output='screen',
        )
```



## List of Available (classes of) Controllers

- There is a list of available controllers (by their class definitions) which we can add to the  **/franka_bringup/config/controllers.yaml** file, using the above method, and then launch as a node by adding it into the launch file
- The ********yaml******** file specifies these controllers to be update at **1000 Hz**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz
```

- This specific ********yaml******** file, we can also customise the ************************panda_joints************************, since they are running **PID** controllers for any commands that are not at the ************effort************ level, and therefore can adjust the **P, I, D gains** to our preference (but again, don’t go too crazy 😎, haven’t tried it yet)
- **Here is the majestic list of controllers:**

controller_manager/test_controller

controller_manager/test_controller_failed_init

controller_manager/test_controller_with_interfaces

diff_drive_controller/DiffDriveController

effort_controllers/GripperActionController

effort_controllers/JointGroupEffortController

force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster

forward_command_controller/ForwardCommandController

forward_command_controller/MultiInterfaceForwardCommandController

franka_example_controllers/GravityCompensationExampleController

franka_example_controllers/JointImpedanceExampleController

franka_example_controllers/MoveToStartExampleController

imu_sensor_broadcaster/IMUSensorBroadcaster

joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller/JointTrajectoryController

position_controllers/GripperActionController

position_controllers/JointGroupPositionController

tricycle_controller/TricycleController

velocity_controllers/JointGroupVelocityController

ackermann_steering_controller/AckermannSteeringController

admittance_controller/AdmittanceController

bicycle_steering_controller/BicycleSteeringController

controller_manager/test_chainable_controller

tricycle_steering_controller/TricycleSteeringController

- ENJOY!



## Creating a custom controller (and export as plugin)

- cd into /**franka_ros2/franka_example_controllers/** (we will be working extensively in this package). Let’s base our custom controller off the “**move_to_start_example_controller**”
- go into **/src** and make a copy of “**move_to_start_example_controller.cpp**”, and rename it to “**my_controller.cpp**”, and change the class name to “**MyController**”
- go into **/include/franka_example_controllers/** and make a copy of “**move_to_start_example_controller.hpp**”, and rename it to “**my_controller.hpp**”, and change the class name to “**MyController**”
- open “**CMakeLists.txt”**, and add “**src/my_controller.cpp**” to the install target
- open “**franka_example_controllers.xml**”, and add our “**MyController”** class as a **new plugin**
- Then, go to **root of** **franka_ws** and **build** the workspace

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=/home/<path_to_your_libfranka_folder>/build
```

- The **order** in which the **controller class functions** are called:
    1. on_init()
    2. on_configure()
    3. command_interface_configuration()
    4. state_interface_configuration()
    5. on_activate()
    6. update()
    7. {updateJointStates() can get called at anytime}
    



## Setting up & launching the custom controller plugin

- cd into /**franka_ros2/franka_bringup/** (we will now be working in this package)
- go into **/config/** and open “**controllers.yaml**”. Add our new controller under the controller manager, and then add the parameters of the controller {**arm_id, k_gains, d_gains**}
- go into **/launch/** and make a copy of “**move_to_start_example_controllers.launch.py**”, and rename it “**my_controller.launch.py**”. Open it, and **change the last node to launch** to the executable of our controller “**my_controller**”.
- Then, go to **root of** **franka_ws** and **build** the workspace

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=/home/<path_to_your_libfranka_folder>/build
```