# [5dpo_ratf_firmware](https://github.com/5dpo/5dpo_ratf_firmware)

This repository implements the firmware relative to the four-wheeled
omnidirectional robotic platform used by the
[5dpo FEUP Robotics Team](https://github.com/5dpo) in the 
[Robot@Factory](https://www.festivalnacionalrobotica.pt/) competition.

**Version 1.0.3**

**With this version, it is possible to do:**

- Serial communication using the library channels
- Read encoders channel A and B for wheeled odometry
- Motors angular speed control with a generic PID controller
- Motors PWM control using the Adafruit Motor Shield V2
- Watchdog timer to disable the motors if the PC does not send anything for a
  certain time
- Limitation on the PWM change
- Reset signal
- Solenoid actuator
- Switch to sense the presence of a box
- Tune the PID controllers for angular speed control of the motors
  (same parameters as the original version used in the competition)
- Fix control period of the motors (100Hz > 50Hz)
- Tune the PID controllers for angular speed control of the motors
- Switch off solenoid actuator when the motors are disabled due to watchdog
  timer

**The next version will add these features:**

No further development is expected for this module.

**Bugs identified in the current version:**

- TBC

## Hardware (TBC)

- 60mm Aluminum Mecanum Wheels Set
  (2x left + 2x right, 45° rotational axis)
- Adafruit Motor Shield V2
- Arduino Mega 2560 Rev3
- DC-DC Buck Converter 300W 6-40V to 1.2-36V 20A Step Dowm
- Grove Electromagnet
- Metal DC Geared Motor w/Encoder - 12V 251RPM 18Kg.cm (DFRobot)
- Raspberry Pi 4
- Raspberry Pi Camera Board v2.1 (8MP, 1080p)
- TATTU 10000mAh 11.1V 3S 15C LiPo Battery
- Waveshare 5inch Capacitive Touch Display (DSI Interface, 800×480)
- YDLIDAR X4 360° Laser Scanner

## Serial Communication (channels)

**Robot > PC**

- `[g]`: encoders total count of motor 0 (ticks)
- `[h]`: encoders total count of motor 1 (ticks)
- `[i]`: encoders total count of motor 2 (ticks)
- `[j]`: encoders total count of motor 3 (ticks)
- `[k]`: interval time between control cycles (us)
- `[r]`: reset signal
- `[s]`: switch (0|1)

**PC > Robot**

- `[G]`: reference angular speed of motor 0 (rad/s)
- `[H]`: reference angular speed of motor 1 (rad/s)
- `[I]`: reference angular speed of motor 2 (rad/s)
- `[J]`: reference angular speed of motor 3 (rad/s)
- `[K]`: PWM value of motors
  - `(value >> 24) & 0x03`: motor index (0..3)
  - `value & 0xFFFF`: PWM value (0..`kMotPWMmax`)
- `[L]`: solenoid (0|1)

## Usage

**Requirements**

- PlatformIO extension for VS Code

### Compilation

1. Clone the repository
   ```sh
   # Clone repository
   git clone git@github.com:5dpo/5dpo_ratf_firmware.git
   cd 5dpo_ratf_firmware

   # Open the folder as a project
   code .
   ```
2. Open `src/main.cpp`
3. Compile the project using the PlatformIO extension in VS Code
   (PlatformIO on left-side navigation bar > Project Tasks > Build)

### Run

1. Open `src/main.cpp`
2. Compile the project using the PlatformIO extension in VS Code
   (PlatformIO on left-side navigation bar > Project Tasks > Upload)
