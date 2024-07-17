# Mr.LfD

This is a central repository for resources and documentation for **Mr.LfD**.

Mr.LfD is an application for the Meta Quest 3 that works with the Franka Emika Panda. It offers features such as teleoperation and visualisation of robot status. You can view the demos [here](https://www.youtube.com/watch?v=FrHReF052ss&list=PLGZ6M30GmbVPnrU4zVaIsvYRqLYsf4KVH&index=11).

Before use, please follow the instructions provided in the [mrlfd_ros2](https://github.com/LOOP115/mrlfd_ros2) and [MrLfD_MQ3](https://github.com/LOOP115/MrLfD_MQ3) repositories.

<br>

## Setup Devices

### Franka Emika Panda

- [Franka Control Interface Documentation](https://frankaemika.github.io/docs/index.html)
  - [franka_ros2](https://frankaemika.github.io/docs/franka_ros2.html)
- [Setup franka_ros2](docs/franka/franka_ros2.md)
- [Setup Gazebo with Franka](docs/franka/gazebo.md)

### Meta Quest 3

- [Get Started with Meta Quest Development in Unity](https://www.youtube.com/watch?v=BU9LYKM2TDc&t=314s)
- [Meta Quest Link](https://www.meta.com/en-gb/help/quest/articles/headsets-and-accessories/oculus-link/set-up-link/)
- [Meta Quest Developer Hub](https://developer.oculus.com/meta-quest-developer-hub/)
- [SideQuest](https://sidequestvr.com/)

<br>

## [mrlfd_ros2](https://github.com/LOOP115/mrlfd_ros2)

This is the ROS2 endpoint for the project, which enables launching of ROS2 programs to control Franka in Gazebo or the real world.

<br>

## [MrLfD_MQ3](https://github.com/LOOP115/MrLfD_MQ3)

This is the Unity endpoint for the project, offering features such as an XR control interface for Franka on Quest 3.

<br>

## [YOLO on Quest3](https://github.com/LOOP115/YOLO_Quest3)

This project utilises the cameras of Quest 3 to identify the shapes and colours of a set of EVA blocks using YOLOv8.

<br>

## [User Study Data](userstudy/)

- [Participant demographics](userstudy/demographics.csv)
- [Feedback from participants on each task](userstudy/tasks.csv)
- [Number of attempts and completion times for each task](userstudy/tasks_data.xlsx)
- [Trajectory data from Franka for each task](userstudy/trajectory/)
  - Naming format: `task<taskId>_<round>_<participantId>.csv`
  - [Kinesthetic teaching](userstudy/trajectory/kinesthetic/)
  - [Teleoperation](userstudy/trajectory/teleoperation/)
- [Zip file containing all data](userstudy/userstudy.zip)

<br>

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.
