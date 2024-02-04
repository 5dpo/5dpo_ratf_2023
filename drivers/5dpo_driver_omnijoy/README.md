# 5dpo_driver_omnijoy

**Version 0.0.0**

This repository implements a ROS driver to control an omnidirectional robot
(but also compatible with other steering geometries) with a joystick controller.

**With this version, it is possible to do:**

- Joystick control of a mobile robot

**The next version will add these features:**

- No further development is foreseen for this package

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [geometry_msgs](https://wiki.ros.org/geometry_msgs)
- [joy](https://wiki.ros.org/joy)

### Parameters

- axis_linear_x (`int = 1`): index number of x linear axis (`/joy.axes`)
- axis_linear_y (`int = 2`): index number of y linear axis (`/joy.axes`)
- axis_angular (`int = 0`): index number of angular linear axis (`/joy.axes`)
- axis_deadman (`int = 4`): index number of button for deadman switch
  (`/joy.buttons`)
- axis_turbo (`int = 5`): index number of button for turbo (`/joy.buttons`)
- axis_turbo_up (`int = 6`): index number of button for increasing boost
  (`/joy.buttons`)
- axis_turbo_down (`int = 7`): index number of button for decreasing boost
  (`/joy.buttons`)
- scale_angular (`double = 0.2`): angular scale to normalize
  [`Joy.msg`](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html)
- scale_linear (`double = 0.1`): linear scale to normalize
  [`Joy.msg`](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html)
- turbo_scale_linear (`double = 0.2`): use this scale instead of `scale_linear`
  when pressing the turbo button
- turbo_max_scale_linear (`double = 0.4`): maximum turbo scale
- turbo_scale_angular (`double = 0.4`): use this scale instead of
  `scale_angular` when pressing the turbo button

In order to know the index numbers of your controller, go to
[joy](https://wiki.ros.org/joy) documentation, _5. Application_, to see the
options available for those parameters.

### Subscribes

- joy
([`Joy.msg`](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html))

### Publishes

- cmd_vel
  ([`Twist.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

### Services

None.

### Actions

None.

## Usage

### Build

```sh
# Create catkin workspace
mkdir -p ~/catkin_ws/src

# Clone repository
cd ~/catkin_ws/src
git clone git@github.com:5dpo/5dpo_driver_omnijoy.git

# Build
cd ..
catkin build
```

### Launch

```sh
roslaunch sdpo_driver_omnijoy sdpo_driver_omnijoy.launch
```

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Fernando Sá ([gitlab](https://gitlab.inesctec.pt/fjms))
- Héber Miguel Sobreira ([gitlab](https://gitlab.inesctec.pt/heber.m.sobreira),
  [inesctec](mailto:heber.m.sobreira@inesctec.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
