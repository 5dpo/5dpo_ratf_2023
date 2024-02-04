# [5dpo_ratf_ros_nav_conf](https://github.com/5dpo/5dpo_ratf_ros_nav_conf)

**Version 0.2.4**

This repository implements the launch files required for the 5DPO Navigation
Stack on the four-wheeled omnidirectional robotic platform used by the
[5dpo FEUP Robotics Team](https://github.com/5dpo) in the
[Robot@Factory](https://www.festivalnacionalrobotica.pt/) competition. The
system implemented is based on the INESC TEC Robotics Navigation Stack that it
allows you to have different configurations implemented and selecting just one
based on your environment variables.

**With this version, it is possible to do:**

- ROS package creation (`CMakeLists.txt`, `package.xml`)
- `basic` configuration
- `ratf2023` configuration

**The next version will add these features:**

- TBD

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [5dpo_ratf_firmware](https://github.com/5dpo/5dpo_ratf_firmware)
- [rviz](https://wiki.ros.org/rviz)
- [sdpo_driver_omnijoy](https://github.com/5dpo/5dpo_driver_omnijoy)
- [sdpo_ratf_ros_driver](https://github.com/5dpo/5dpo_ratf_ros_driver/)
- [sdpo_ros_odom](https://github.com/5dpo/5dpo_ros_odom)

## Usage

### Configurations

**Usage**

```sh
# Robot id
export ROBOT_ID=<id>                # (default: unnamed_robot)
# Configuration
export ROBOT_CONF=<configuration>   # (default: basic)
```

**`basic`**

- Drivers
    - sdpo_driver_omnijoy
    - sdpo_ratf_ros_driver
- Human-Machine Interface (HMI)
    - rviz
- Localization
    - sdpo_ros_odom

### Compilation

```sh
# Create catkin workspace
mkdir -p ~/catkin_ws/src

# Clone repository
cd ~/catkin_ws/src
git clone git@github.com:5dpo/5dpo_ratf_ros_nav_conf.git

# Build
cd ..
catkin build
```

### Launch

```sh
roslaunch sdpo_ratf_ros_nav_conf wake_up_almighty_ratf.launch
```

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Héber Miguel Sobreira ([gitlab](https://gitlab.inesctec.pt/heber.m.sobreira),
  [inesctec](mailto:heber.m.sobreira@inesctec.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
