# 5dpo_ros_interfaces_hw

**Version 2.1.0**

This repository contains a ROS package relative to all hardware interfaces
required to run the 5dpo ROS robotics stack. The hardware interfaces defined in
this repository are the following ones:

- [mot_enc_array.msg](msg/mot_enc_array.msg)
- [mot_ref.msg](msg/mot_ref.msg)
- [mot_data_array.msg](msg/mot_data_array.msg)

**With this version, it is possible to do:**

- Hardware interface to retrieve the wheels encoders data
- Hardware interface to set the wheels angular speed reference
- Hardware interface to retrieve the wheels motors data

**The next version will add these features:**

- TBD

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [message_generation](https://wiki.ros.org/message_generation)
- [message_runtime](https://wiki.ros.org/message_runtime)
  (_runtime_)

### Interfaces

**[mot_enc_array.msg](msg/mot_enc_array.msg)**

- stamp: time stamp of the message
- mot_enc_array_data: array containing the encoders data
  - encoder_delta: number of encoder ticks between two time instants
    (ticks/time interval)
  - ticks_per_rev: total number of encoder ticks equivalent to a wheel
    revolution (ticks/rot)
  - angular_speed: wheel angular speed (rad/s)

**[mot_ref.msg](msg/mot_ref.msg)**

- angular_speed_ref: wheel angular speed reference (rad/s)

**[mot_data_array.msg](msg/mot_data_array.msg)**

- stamp: time stamp of the message
- mot_enc_array_data: array containing the encoders data
  - sample_period: sample period of the motors data (s)
  - pwm: pwm integer value set for the motor drivers
  - encoder_delta: number of encoder ticks between two time instants
    (ticks/time interval)
  - ticks_per_rev: total number of encoder ticks equivalent to a wheel
    revolution (ticks/rot)
  - angular_speed: wheel angular speed (rad/s)

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
