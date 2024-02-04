# [5dpo_ratf_ros_driver](https://github.com/5dpo/5dpo_ratf_ros_driver)

This repository implements a driver within a ROS package to communicate with the
firmware present in the four-wheeled omnidirectional robotic platform used by
the [5dpo FEUP Robotics Team](https://github.com/5dpo) in the
[Robot@Factory](https://www.festivalnacionalrobotica.pt/) competition. The
driver is required for communicating with the robot and have available all its
different functions.

The serial communication is handled by
[Boost.Asio](https://www.boost.org/doc/libs/1_80_0/doc/html/boost_asio.html).
This communication is based on the example `4_callback` provided in the
[serial-port](https://github.com/fedetft/serial-port) GitHub repository.

**Version 1.4.0**

**With this version, it is possible to do:**

- Communicate with Arduino Mega 2560 using Boost.Asio
  ([sdpo_ros_serial_port](https://github.com/5dpo/5dpo_ros_serial_port))
- Subscribe motors angular speed reference
- Publish encoders data (encoders + wheels angular speed)
- Read encoders
- Set motors speed
- Reset driver upon reset signal
- Watchdog timer to monitor the motors angular speed reference
- Send serial message to the firmware upon reconnection of the serial port
  communication
- Read switch
- Set solenoid
- Node specific for setting PWM values through ROS services and log data for
  future calibration of the PI controllers
- Scripts (see in [sh](sh/)) to automate the retrieval of log data for tunning
  the PI controllers
- Synchronization of the firmware with the ROS publication topics

**The next version will add these features:**

- Publish optionally the odometry data

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [message_generation](https://wiki.ros.org/message_generation)
- [message_runtime](https://wiki.ros.org/message_runtime)
- [sdpo_ros_interfaces_hw](https://github.com/5dpo/5dpo_ros_interfaces)
- [sdpo_ros_serial_port](https://github.com/5dpo/5dpo_ros_serial_port)
- [serial_communication_channels](https://github.com/5dpo/serial_communication_channels)
- [std_msgs](https://wiki.ros.org/std_msgs)
- [std_srvs](https://wiki.ros.org/std_srvs)

### Parameters

- enc_ticks_per_rev (`float = 64.0`): resolution of the encoder (ticks/rot)
- gear_reduction (`float = 43.8`): reduction ratio of the transmissions
  (\[gear_reduction:1\])
- serial_port_name (`std::string = "/dev/ttyACM0"`): name of the serial port

### Subscribes

- motors_ref
  ([mot_ref.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_ref.msg))

### Publishes

- motors_encoders
  ([mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg))
- switch_state
  ([Bool.msg](https://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))

**sdpo_ratf_ros_driver_tune**

- motors_data
  ([mot_data_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_data_array.msg))

### Services

- set_solenoid_state
  ([SetBool.srv](https://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html))

**sdpo_ratf_ros_driver_tune**

- set_motors_pwm ([SetMotorsPWM.srv](srv/SetMotorsPWM.srv))
  ```sh
  rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
  ```

### Actions

None.

## Usage

### Build

```sh
# Create catkin workspace
mkdir -p ~/catkin_ws/src

# Clone repository
cd ~/catkin_ws/src
git clone git@github.com:5dpo/5dpo_ratf_ros_driver.git

# Build
cd ..
catkin build
```

### Launch

**sdpo_ratf_ros_driver_node**

```sh
roslaunch sdpo_ratf_ros_driver sdpo_ratf_ros_driver.launch
```

**sdpo_ratf_ros_driver_tune**

1. Measure the battery level with a multimeter
2. Open a terminal and launch the node
   ```sh
   roslaunch sdpo_ratf_ros_driver sdpo_ratf_ros_driver_tune.launch battery:=<battery level>
   ```
3. Open another terminal and use the shell scripts provided in this repository
   - [set_robot_pwm.sh](sh/set_robot_pwm.sh)
     ```sh
     ./set_robot_pwm.sh <v,vn,w> <pwm value>
     ```
     - w / #ticks = f(PWM)
     - E.g., estimate the deadzone of the motor and tune the Hammerstein
       nonlinear block implemented in the firmware
   - [auto_tunning_motion.sh](sh/auto_tunning_motion.sh)
     ```sh
     ./auto_tunning_motion.sh <#runs per motion> <time per motion state (s)> <v,vn: pwm ini> <v,vn: pwm fin> <w: pwm ini> <w: pwm fin>
     ```
     - w / #ticks = f(PWM)
     - Shell script to automatically perform v, vn, w motions to use the logged
       data for tunning the PI controller with, e.g., the _Internal Model_
       _Control (IMC) method_

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
