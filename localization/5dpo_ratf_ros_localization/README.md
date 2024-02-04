# [5dpo_ratf_ros_localization](https://github.com/5dpo/5dpo_ratf_ros_localization)

This repository implements the localization module for the four-wheeled
omnidirectional robotic platform used by the
[5dpo FEUP Robotics Team](https://github.com/5dpo) in the
[Robot@Factory](https://www.festivalnacionalrobotica.pt/) competition. The
module implements an Extended Kalman Filter (EKF) to fuse two data sources:
odometry and beacons position (the latter detected using a 2D laser scanner).

**Version 1.3.0**

**With this version, it is possible to do:**

- EKF predict step for wheeled odometry
- EKF update step for beacons detected with a 2D laser scanner
- tf publication of the filter
- tf initialization
- EKF execution mode (wheeled odometry only, sensors only, fusion)
- [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure) for setting
  the filter execution mode (wheeled odometry only, sensors only, fusion)
- [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure) for setting
  the covariances of the filter
- Publish estimated pose of the filter

**The next version will add these features:**

- TBD

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure)
- [Eigen3](https://eigen.tuxfamily.org/)
- [geometry_msgs](https://wiki.ros.org/geometry_msgs)
- [nav_msgs](https://wiki.ros.org/nav_msgs)
- [sensor_msgs](https://wiki.ros.org/sensor_msgs)
- [tf](https://wiki.ros.org/tf)
- [visualization_msgs](https://wiki.ros.org/visualization_msgs)

### Parameters

- base_frame_id (`std::string = "base_footprint"`): tf frame id of the robot
  base footprint coordinate frame
- laser_frame_id (`std::string = "laser"`): tf frame id of the 2D laser scanner
  coordinate frame
- odom_frame_id (`std::string = "odom"`): tf frame id of the robot odometry
  coordinate frame
- map_frame_id (`std::string = "map"`): tf frame id of the map coordinate
  frame
- beacons_diam (`double = 0.09`): beacon diameter (m)
- beacons_valid_dist (`double = 0.2`): valid distance between observed laser
  point and an existent beacon (m)
- beacons (`matrix`): 2D position of each beacon in the map coordinate frame (m)
- ekf_pose_ini_x (`double = 0.0`): initial x position of the EKF (m)
- ekf_pose_ini_y (`double = 0.0`): initial y position of the EKF (m)
- ekf_pose_ini_th (`double = 0.0`): initial yaw orientation of the EKF (deg)
- ekf_cov_ini_p_x (`double = 0.1`): initial state covariance for x position
- ekf_cov_ini_p_y (`double = 0.1`): initial state covariance for y position
- ekf_cov_ini_p_th (`double = 0.1`): initial state covariance for orientation
- ekf_cov_q_d (`double = 0.1`): state transition covariance in the x direction
  of the robot coordinate frame
- ekf_cov_q_dn (`double = 0.1`): state transition covariance in the y direction
  of the robot coordinate frame
- ekf_cov_q_dth (`double = 0.05`): state transition covariance in the angular
  direction of the robot coordinate frame
- ekf_cov_r_dist (`double = 0.0001`): observation covariance for laser sampled
  distance
- ekf_cov_r_ang (`double = 0.001`): observation covariance for laser sampled
  angle
- ekf_mode_ini (`std::string = "Fusion"`): initial execution mode for the EKF
  (`"OdomWhOnly"|"SensOnly"|"Fusion"`)
- publish_pose (`bool = false`): publish estimated pose of the EKF

### Subscribes

- initial_pose
  ([geometry_msgs/PoseWithCovarianceStamped.msg](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
- laser_scan_point_cloud
  ([sensor_msgs/PointCloud.msg](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud.html))
- odom
  ([nav_msgs/Odometry.msg](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))
- tf (N/A)
  - base_footprint_frame_id > odom_frame_id

### Publishes

- beacons
  ([sensor_msgs/PointCloud.msg](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud.html))
- map_beacons
  ([visualization_msgs/MarkerArray.msg](http://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html))
- pose
  ([geometry_msgs/PoseStamped.msg](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html))
- tf (N/A)
  - odom_frame_id > map_frame_id

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
git clone git@github.com:5dpo/5dpo_ratf_ros_localization.git

# Build
cd ..
catkin build
```

### Launch

```sh
roslaunch sdpo_ratf_ros_localization sdpo_ratf_ros_localization.launch
```

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Héber Miguel Sobreira ([gitlab](https://gitlab.inesctec.pt/heber.m.sobreira),
  [inesctec](mailto:heber.m.sobreira@inesctec.pt))
- João G. Martins ([github](https://github.com/Joao-G-Martins),
  [feup](mailto:up201806222@edu.fe.up.pt),
  [inesctec](mailto:joao.g.martins@inesctec.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
