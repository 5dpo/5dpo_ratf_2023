# [5dpo_driver_ros_camera](https://github.com/5dpo/5dpo_driver_ros_camera)

This package contains the camera driver with Aruco pose estimation designed for
use in the 2023 5DPO Robot@Factory robots.

**Version 0.0.0**

**With this version, it is possible to do:**

- TBC

**The next version will add these features:**

- TBC

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure)
- [geometry_msgs](https://wiki.ros.org/geometry_msgs)
- [message_generation](https://wiki.ros.org/message_generation)
- [message_runtime](https://wiki.ros.org/message_runtime)
- [roslib](https://wiki.ros.org/roslib)
- [std_msgs](https://wiki.ros.org/std_msgs)
- [tf](https://wiki.ros.org/tf)

### Parameters

To obtain the camera's intrinsic parameters, please refer to the following
tutorial:
[OpenCV Camera Calibration](https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html). Place the generated file in the
[config](config/) folder and ensure the proper file name is placed in the launch
file.

- debug (`bool = false`): enable/disable additional messages and showing the
  camera image
- publish_rate (`int = 20`): publication rate of the arucos perceived by the
  camera (Hz)
- base_frame_id (`string = "base_footprint"`): tf frame id of the robot base
  footprint coordinate frame
- camera_frame_id (`string = "camera"`): tf frame id of the camera coordinate
  frame
- camera_pose_x (`float = 0.0`): x of the camera pose relative to the robot base
  footprint coordinate frame (m)
- camera_pose_y (`float = 0.0`): y of the camera pose relative to the robot base
  footprint coordinate frame (m)
- camera_pose_z (`float = 0.0`): z of the camera pose relative to the robot base
  footprint coordinate frame (m)
- camera_pose_yaw (`float = 0.0`): yaw angle of the camera pose relative to the
  robot base footprint coordinate frame (deg)
- camera_pose_pitch (`float = 0.0`): pitch angle of the camera pose relative to
  the robot base footprint coordinate frame (deg)
- camera_pose_roll (`float = 0.0`): roll angle of the camera pose relative to
  the robot base footprint coordinate frame (deg)

### Subscribes

None.

### Publishes

- arucos
  ([sdpo_driver_ros_camera/ArucoArray.msg](msg/ArucoArray.msg))
- tf (N/A)
  - camera_frame_id > base_frame_id

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
git clone git@github.com:5dpo/5dpo_driver_ros_camera.git

# Build
cd ..
catkin build
# OR catkin_make_isolated (more slow, build and check dependencies individually)
# OR catkin build (requires catkin tools)
```

### Launch

TBC

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- João Costa ([github](https://github.com/costajoao1641/),
  [mail:student](mailto:up201806431@fe.up.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.inesctec.pt/ricardo.b.sousa),
  [mail:inesctec](mailto:ricardo.b.sousa@inesctec.pt),
  [mail:personal](mailto:sousa.ricardob@outlook.com),
  [mail:professor](mailto:rbs@fe.up.pt),
  [mail:student](mailto:up201503004@edu.fe.up.pt))
- Vítor Ventuzelos ([github](https://github.com/BerserkingIdiot),
  [gitlab](https://gitlab.inesctec.pt/vitor.ventuzelos),
  [mail:inesctec](mailto:vitor.ventuzelos@inesctec.pt),
  [mail:personal](mailto:skullventuzelos@hotmail.com))
