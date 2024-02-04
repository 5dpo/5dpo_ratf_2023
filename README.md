# 5dpo_ratf_2023

System framework for the Robot@Factory 4.0 competition
([info](https://www.festivalnacionalrobotica.pt/2023/robotfactory-4-0/),
[GitHub](https://github.com/P33a/RobotAtFactory/tree/main)) hosted at the
[Portuguese Robotics Open (FNR)](https://www.festivalnacionalrobotica.pt/).
This framework is the lastest version developed by the
[5dpo Robotics Team](https://5dpo.github.io/) and used in the 2023 edition of
the competition.
Furthermore, the robot is a four-wheeled omnidirectional platform also developed
in the scope of the competition. All the information about the electrical and
mechanical components can be found below.
Finally, the launch file system implemented is bsed on the
[INESC TEC](https://www.inesctec.pt/en) Robotics Navigation Stack that allows
you to have different configurations implemented and selecting just one based on
your environment variables.

## ROS

**Current version**
- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

**Nodes**
- Drivers:
  - [sdpo_ratf_ros_driver](/drivers/5dpo_ratf_ros_driver/) (driver to
    communicate to the firmware running in the Arduino Mega 2560)
  - [sdpo_driver_omnijoy](/drivers/5dpo_driver_omnijoy/) (joystick driver
    compatible with omnidirectional platforms)
  - [sdpo_driver_laser_2d](/drivers/5dpo_driver_laser_2d/) (driver to
    communicate through a serial connection with the YDLIDAR X4)
  - [sdpo_driver_ros_camera](/drivers/5dpo_driver_ros_camera/) (camera driver
    to publish the poses of ArUco markers) _(NOT STABLE!)_
- Localization:
  - [sdpo_ros_odom](/localization/5dpo_ros_odom/) (wheeled odometry relative
    pose estimation)
  - [sdpo_ratf_ros_localization](/localization/5dpo_ratf_ros_localization/)
    (beacon-based Extended Kalman Filter to estimate the pose of the robot)
- Navigation:
  - [sdpo_ratf_ros_path_planning](/navigation/5dpo_ratf_ros_path_planning/)
    (trajectory controller and path planning to execute the competition rounds)
- Navigation Configuration:
  - [sdpo_ratf_ros_nav_conf](/5dpo_ratf_ros_nav_conf/) (launch file system)

## Usage

### Configurations

- `basic`
  - Drivers
    - [sdpo_ratf_ros_driver](/drivers/5dpo_ratf_ros_driver/)
    - [sdpo_driver_omnijoy](/drivers/5dpo_driver_omnijoy/)
  - Human-Machine Interface (HMI)
    - rviz
  - Localization
    - [sdpo_ros_odom](/localization/5dpo_ros_odom/)
- `ratf2023`
  - Drivers
    - [sdpo_ratf_ros_driver](/drivers/5dpo_ratf_ros_driver/)
    - [sdpo_driver_omnijoy](/drivers/5dpo_driver_omnijoy/)
    - [sdpo_driver_laser_2d](/drivers/5dpo_driver_laser_2d/)
  - Human-Machine Interface (HMI)
    - rviz
  - Localization
    - [sdpo_ros_odom](/localization/5dpo_ros_odom/)
    - [sdpo_ratf_ros_localization](/localization/5dpo_ratf_ros_localization/)
  - Navigation
    - [sdpo_ratf_ros_path_planning](/navigation/5dpo_ratf_ros_path_planning/)

**Usage**

```sh
# Robot id
export ROBOT_ID=<id>                # (default: unnamed_robot)
# Configuration
export ROBOT_CONF=<configuration>   # (default: basic)
```

### Build

```sh
```

### Launch

```sh
```
