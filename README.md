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

## Hardware

- Mechanics:
  - 60mm Aluminum Mecanum Wheels Set (2x left + 2x right, 45° rotational axis)
    ([shop](https://eu.robotshop.com/products/60mm-mecanum-wheel-set-2x-left-2x-right))
  - Metal DC Geared Motor w/Encoder - 12V 251RPM 18Kg.cm (DFRobot)
    ([shop-1](https://eu.robotshop.com/products/12v-dc-motor-251rpm-encoder),
    [shop-2](https://www.dfrobot.com/product-634.html))
- Computing Units:
  - Arduino Mega 2560 Rev3
    ([shop](https://store.arduino.cc/products/arduino-mega-2560-rev3))
  - Raspberry Pi 4B 4GB
    ([shop-1](https://mauser.pt/catalog/product_info.php?products_id=096-7402),
    [shop-2](https://pt.farnell.com/raspberry-pi/rpi4-modbp-4gb/raspberry-pi-4-model-b-4gb/dp/3051887),
    [shop-3](https://pt.mouser.com/ProductDetail/Raspberry-Pi/SC01949?qs=T%252BzbugeAwjjISb%252BwlagpRw%3D%3D),
    [shop-4](https://www.digikey.pt/en/products/detail/raspberry-pi/SC0194-9/10258781?s=N4IgTCBcDaIIwDYCcAGAtAJQIIGUAKAQgKIYYCaeAkgCwED01A4gWgHIAiIAugL5A))
- Electronics:
  - DC-DC Buck Converter 300W 6-40V to 1.2-36V 20A Step Dowm
    ([shop](https://www.amazon.com/DIANN-300W-DC-DC-Buck-Converter/dp/B0B4CZ6GRV))
  - Grove Electromagnet
    ([shop](https://eu.robotshop.com/products/grove-electromagnet))
  - LiPo Battery 11.1V 3S
    - Gens ace 5000mAh 11.1V 3S1P 60C Lipo Battery Pack with XT90 Plug Bashing
      Series
      ([shop](https://www.gensace.de/gens-ace-5000mah-11-1v-3s1p-60c-lipo-battery-pack-with-xt-90-plug-bashing-series.html))
    - Tattu 11.1V 15C 3S 10000mAh Lipo Battery Pack
      ([shop](https://www.gensace.de/tattu-11-1v-15c-3s-10000mah-lipo-battery-pack.html))
  - Micro Switch with Roller Lever Arm
    ([shop](https://mauser.pt/catalog/product_info.php?products_id=010-1473))
  - Power ON/OFF Switch
    ([shop](https://mauser.pt/catalog/product_info.php?products_id=010-0626))
- Sensors:
  - 10 DoF Inertial Measurement Unit (IMU)
    ([shop](https://www.waveshare.com/10-dof-imu-sensor-b.htm))
  - Raspberry Pi Camera Board v2.1 (8MP, 1080p)
    ([shop-1](https://mauser.pt/catalog/product_info.php?products_id=096-4061),
    [shop-2](https://pt.farnell.com/raspberry-pi/rpi-8mp-camera-board/raspberry-pi-camera-board-v2/dp/2510728),
    [shop-3](https://pt.mouser.com/ProductDetail/Raspberry-Pi/SC0023?qs=T%252BzbugeAwjgRU4vb4%252BbLIg%3D%3D),
    [shop-4](https://www.digikey.pt/en/products/detail/raspberry-pi/SC0023/6152810?s=N4IgTCBcDaIIwDYCcAGAtHFc5oHIBEQBdAXyA))
  - YDLIDAR X4 360° Laser Scanner
    ([shop](https://eu.robotshop.com/products/ydlidar-x4-360-laser-scanner))

## Firmware

- Arduino Mega 2560 Rev3 ([doc](https://docs.arduino.cc/hardware/mega-2560/))
- PlatformIO ([url](https://platformio.org/),
  [vscode](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide),
  [clion](https://plugins.jetbrains.com/plugin/13922-platformio-for-clion))
  - Pre-requisite:
    ```sh
    sudo apt install python3-venv
    ```
  - Setup udev rules
    ([source](https://docs.platformio.org/en/stable/core/installation/udev-rules.html)):
    ```sh
    curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules

    sudo service udev restart
    sudo usermod -a -G dialout $USER
    sudo usermod -a -G plugdev $USER
    ```
  - If you want to define a fixed name to the serial port connection, add the
    following lines to a file `/etc/udev/rules.d/99-usb-serial.rule`
    ([source](https://roboticsknowledgebase.com/wiki/tools/udev-rules/)):
    ```conf
    SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="ttyArduinoMega"
    ```
    - _Note:_ this approach creates a symbolic link, but the original device
      still appears in the `/dev/` folder
- Implementation: [5dpo_ratf_firmware](/firmware/5dpo_ratf_firmware/)

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

If you want to set up a persistent `ROBOT_ID` environment variable, add the
following line to the file `/etc/environment`:

```sh
ROBOT_ID=<id>
```

### Build

```sh
mkdir ~/ros1_ws/src -p
cd ~/ros1_ws/src

git clone git@github.com:5dpo/5dpo_ratf_2023.git

cd ../
catkin_make --force-cmake -DCMAKE_BUILD_TYPE=Release
```

### Launch

```sh
export ROBOT_CONF=<basic|ratf2023>
roslaunch sdpo_ratf_ros_nav_conf wake_up_almighty_ratf.launch
```

## License

Distributed under the _MIT License_.
See [LICENSE](/LICENSE) for more information.

## References

TBC.

## Contacts

- António Paulo Moreira ([github](https://github.com/apaulomoreira),
  [mail](mailto:amoreira@fe.up.pt))
- Cláudia Daniela Rocha ([github](https://github.com/rochaclaudia),
  [mail](mailto:claudia.d.rocha@inesctec.pt))
- João G. Martins ([github](https://github.com/Joao-G-Martins),
  [mail:student](mailto:up201806222@edu.fe.up.pt),
  [mail:inesctec](mailto:joao.g.martins@inesctec.pt))
- João Pedro Costa ([github](https://github.com/costajoao1641),
  [mail](mailto:up201806431@edu.fe.up.pt))
- João Tomás Padrão ([github](https://github.com/JoaoP4dr4o),
  [mail](mailto:up202108766@edu.fe.up.pt))
- José Maria Sarmento ([github](https://github.com/JoseMQS),
  [mail](mailto:jose.m.sarmento@inesctec.pt))
- José Pedro Carvalho ([github](https://github.com/Jose-PCarvalho),
  [mail](mailto:jpcarvalho@fe.up.pt))
- Maria Silva Lopes ([github](https://github.com/mariaslopes),
  [mail:student](mailto:up201806775@edu.fe.up.pt),
  [mail:inesctec](mailto:maria.s.lopes@inesctec.pt))
- Paulo José Gomes da Costa ([github](https://github.com/P33a),
  [mail](mailto:paco@fe.up.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.inesctec.pt/ricardo.b.sousa),
  [mail:inesctec](mailto:ricardo.b.sousa@inesctec.pt),
  [mail:personal](mailto:sousa.ricardob@outlook.com),
  [mail:student](mailto:up201503004@edu.fe.up.pt))

Project Link:
https://github.com/5dpo/5dpo_ratf_2023/.

## Acknowledgements

- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [Electrical and Computers Engineering Department (DEEC) @ FEUP](https://sigarra.up.pt/feup/en/UNI_GERAL.UNIDADE_VIEW?pv_unidade=13)
- [CRIIS - Centre for Robotics in Industry and Intelligent Systems](https://www.inesctec.pt/en/centres/criis/) from
  [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)

## Funding

TBC.
