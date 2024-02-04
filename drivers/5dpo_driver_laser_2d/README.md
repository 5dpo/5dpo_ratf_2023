# [5dpo_driver_laser_2d](https://github.com/5dpo/5dpo_driver_laser_2d)

This repository focus on developing a driver for communicating with
[RPLIDAR](https://www.slamtec.com/en) and variants (specifically,
[YDLIDAR](https://www.ydlidar.com/)) of 2D laser scanners.
The main purpose is to be able to compensate in the future the linear and
angular velocity in the laser data.

**Version 2.4.0**

**With this version, it is possible to do:**

- [YDLIDAR X4](https://www.ydlidar.com/products/view/5.html) support
- [RPLIDAR S2](https://www.slamtec.com/en/S2) support
- tf publication of the laser pose relative to the robot's coordinate frame
- Library available to use the lasers outside of this ROS package
- Dynamic reconfiguration of the laser pose relative to the base footprint
  coordinate frame

**The next version will add these features:**

- [RPLIDAR S2](https://www.slamtec.com/en/S2) dense mode

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [rospy](https://wiki.ros.org/rospy)
- [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure)
- [sdpo_ros_serial_port](https://github.com/5dpo/5dpo_ros_serial_port)
- [sensor_msgs](https://wiki.ros.org/sensor_msgs)
- [tf](https://wiki.ros.org/tf)

### Parameters

- model (`std::string`): model of the 2D laser (`"rplidars2" | "ydlidarx4"`)
- serial_port_name (`std::string = "/dev/ttyUSB0"`): name of the serial port
  - **Note:** do not forget to execute the following commands in the terminal:
    ```shell
    # Add user to group
    sudo usermod -a -G dialout $USER
    sudo usermod -a -G plugdev $USER

    # Update permissions of the serial port (a - all users + rw - read&write)
    sudo chmod a+rw /dev/ttyUSB0
    ```
- baud_rate (`unsigned int`): baud rate of the serial connection (bps)
  - [YDLIDAR X4](https://www.ydlidar.com/products/view/5.html): `115200`
  - [RPLIDAR S2](https://www.slamtec.com/en/S2): `1000000`
- base_frame_id (`std::string = "base_footprint"`): tf frame id of the robot
  base footprint coordinate frame
- laser_frame_id (`std::string = "laser"`): tf frame id of the 2D laser scanner
  coordinate frame
- laser_pose_x (`float = 0.0`): x of the laser pose relative to the robot base
  footprint coordinate frame (m)
- laser_pose_y (`float = 0.0`): y of the laser pose relative to the robot base
  footprint coordinate frame (m)
- laser_pose_z (`float = 0.0`): z of the laser pose relative to the robot base
  footprint coordinate frame (m)
- laser_pose_yaw (`float = 0.0`): yaw angle of the laser pose relative to the
  robot base footprint coordinate frame (deg)
- laser_pose_pitch (`float = 0.0`): pitch angle of the laser pose relative to
  the robot base footprint coordinate frame (deg)
- laser_pose_roll (`float = 0.0`): roll angle of the laser pose relative to the
  robot base footprint coordinate frame (deg)
- dist_min (`float`): minimum distance range (m)
- dist_max (`float`): maximum distance range (m)
- angle_min (`float`): minimum angle range (deg, `[-180.0, 180.0[`)
- angle_max (`float`): maximum angle range (deg, `[-180.0, 180.0[`)
  - **Note:** if a minimum or a maximum value is set, both limits must be
    defined!

### Subscribes

None.

### Publishes

- laser_scan_point_cloud
  ([PointCloud.msg](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud.html))
- tf (N/A)
  - laser_frame_id > base_frame_id

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
git clone git@github.com:5dpo/5dpo_driver_laser_2d.git

# Build
cd ..
catkin build
# OR catkin_make_isolated (more slow, build and check dependencies individually)
# OR catkin build (requires catkin tools)
```

### Launch

**[YDLIDAR X4](https://www.ydlidar.com/products/view/5.html)**

```sh
roslaunch sdpo_driver_laser_2d sdpo_driver_laser_2d_YDLIDARX4.launch
```

**[RPLIDAR S2](https://www.slamtec.com/en/S2)**

```sh
roslaunch sdpo_driver_laser_2d sdpo_driver_laser_2d_RPLIDARS2.launch
```

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Héber Miguel Sobreira ([github](https://github.com/HeberSobreira),
  [gitlab](https://gitlab.inesctec.pt/heber.m.sobreira),
  [mail](mailto:heber.m.sobreira@inesctec.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.inesctec.pt/ricardo.b.sousa),
  [mail:inesctec](mailto:ricardo.b.sousa@inesctec.pt),
  [mail:personal](mailto:sousa.ricardob@outlook.com),
  [mail:professor](mailto:rbs@fe.up.pt),
  [mail:student](mailto:up201503004@edu.fe.up.pt))
