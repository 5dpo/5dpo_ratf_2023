# 5dpo_ros_interfaces

This repository contains the ROS interfaces (actions, messages, services)
common and required to run the 5dpo ROS robotics stack.
Most of the interfaces are based on the used ones in INESC TEC Robotics
Navigation Stack
([gitlab](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack)).

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](http://wiki.ros.org/noetic)

## Usage

### Compilation

```sh
# Create catkin workspace
mkdir -p ~/catkin_ws/src

# Clone repository
cd ~/catkin_ws/src
git clone git@github.com:5dpo/5dpo_ros_interfaces.git

# Build
cd ~/catkin_ws
catkin build
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
