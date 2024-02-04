# Raspberry Pi 4 Model B

## Operating System (OS)

**Raspberry OS (Legacy) (Debian Buster)**

1. Download and install Raspberry Pi Imager
   (https://www.raspberrypi.com/software/)
   - If you have problems with dependencies, execute the following commands:
     ```sh
     sudo apt install qml-module-qtquick-controls2 qml-module-qtquick-templates2 -y
     sudo apt --fix-broken install
     ```
   - Then, execute the following command:
     ```sh
     sudo dpkg -i imager_1.7.4_amd64.deb
     ```
   - Another possibility is installing Raspberry Pi Imager through Snap
     ```sh
     sudo snap install rpi-imager
     ```
2. Open Raspberry Pi Imager
   - Open a terminal (Ctrl + Alt + T)
   - Execute the following command:
     ```sh
     rpi-imager
     ```
3. Select the following items and click Save:
   - Operating System: _Raspberry Pi OS (other) > Raspberry Pi OS (Legacy)_
     _(Debian Buster)_
     - [Robot Operating System (ROS)](https://www.ros.org/) only works with
       Debian Buster-based distros (i.e., Debian 10)
     - Source:
       http://wiki.ros.org/noetic/Installation/Debian#noetic.2FInstallation.2FDebianSources.Setup_your_sources.list
   - Storage: _Generic-USB3.0_CRW_-SD - 30.9GB_
   - Advanced options:
     - Set hostname: _sdpo-ratf_
     - Enable SSH > Use password authentication
     - Set username and password:
       - Username: _sdpo-ratf_
       - Password: _5dpo5dpo_
     - Set locale settings:
       - Time zone: _Europe/Lisbon_
       - Keyboard layout: _pt_
4. Write to flash the SD card

## Utilities

**Terminator**

```sh
sudo apt update
sudo apt install terminator
```

**Git**

```sh
sudo apt update
sudo apt install git
git config --global user.name "5dpo"
git config --global user.email "5dpomsl@gmail.com"
```

**Visual Studio Code**

1. Download Visual Studio Code .deb file
   (https://code.visualstudio.com/#alt-downloads)
   - Select Arm64 .deb version
2. Install Visual Studio Code
   ```sh
   cd Downloads
   sudo dpkg -i code_<version>_arm64.deb
   # OR sudo apt install ./code_<version>_arm64.deb
   ```

See the instructions in https://code.visualstudio.com/docs/setup/raspberry-pi to
optimize the application running in RPi.

_Note:_ `sudo apt install code` does not work in Ubuntu Server due to this
particular being only available in Raspbian.

**Visual Studio Code (Extensions)**

- PlatformIO
  (https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
- C++ Extension Pack
  (https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack)
- ROS
  (https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)

**SSH GitHub Keys**

1. Generate an SSH key pair
   ```sh
   ssh-keygen -t ed25519 -C "<comment>"
   Generating public/private ed25519 key pair.
   Enter file in which to save the key (/home/user/.ssh/id_ed25519): /home/user/.ssh/id_ed25519_GitHub
   ...
   ```
2. Configure SSH to point to a different directory
   ```sh
   eval $(ssh-agent -s)
   ssh-add <directory to private SSH key>
   ```
3. Save settings in the `~/.ssh/config`
   ```sh
   cd ~/.ssh/
   touch config
   code config
   ```
   ```txt
   Host GitHub github.com
     HostName github.com
     IdentityFile ~/.ssh/id_ed25519_GitHub
     User 5dpomsl
   ```

(Source: https://docs.gitlab.com/ee/user/ssh.html)

**NoMachine**

1. Download NoMachine (https://www.nomachine.com/)
   - Select _Other operating systems_
   - Download ARMv7 packages (checked with `uname -m` > armv7l)
2. Execute the following commands:
   ```sh
   cd Downloads
   sudo apt install ./nomachine_<version>_armhf.deb
   ```

(Source: https://downloads.nomachine.com/linux/?id=30&distro=Arm)

## Robot Operating System (ROS)

- **Version:** ROS Noetic Ninjemys (Ubuntu Focal 20.04 LTS, Debian Buster 10)
- **Instructions:** http://wiki.ros.org/noetic/Installation/Source (installing
  from source)

### Installation

1. Setup your sources.list (Debian Buster 10)
   ```sh
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt update
   ```
2. Install bootstrap dependencies
   ```sh
   sudo apt install python3-rosdep python3-rosinstall-generator python3-vcstools python3-vcstool python3-empy build-essential
   sudo rosdep init
   rosdep update
   ```
3. Install Python dependencies
   ```sh
   pip3 install importlib-metadata
   pip3 install rospkg
   pip3 install rospkg rosdistro
   pip3 install empy
   ```
4. Create a catkin workspace
   ```sh
   mkdir -p ~/ros_catkin_ws/src
   cd ~/ros_catkin_ws
   ```
5. Download the source code for ROS packages
   ```sh
   rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-desktop.rosinstall
   vcs import --input noetic-desktop.rosinstall ./src
   ```
   - Do not install the Desktop Full variant
     ([REP 150](https://www.ros.org/reps/rep-0150.html)), i.e., `desktop_full`
   - The `desktop_full` adds the simulator Gazebo not compatible with the
     Raspberry Pi
6. Build the catkin workspace
   ```sh
   rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y
   ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
   ```
7. Source the `setup.bash` file
   ```sh
   mkdir -p ~/5dpo_catkin_ws/src
   cd 5dpo_catkin_ws
   catkin_make
   cd
   sudo nano .bashrc
   # > Add the following lines:
   # source ~/ros_catkin_ws/install_isolated/setup.bash
   # source ~/5dpo_catkin_ws/devel/setup.bash
   # > Save the file: Ctrl + O (write), Ctrl + X (exit)
   ```

## OpenCV

- **Version:** 4.7.0 (requires the latest version for Aruco detection and
  processing)
- **Instructions:** https://docs.opencv.org/4.7.0/d7/d9f/tutorial_linux_install.html

### Installation

```sh
# Install minimal prerequisites (Ubuntu 18.04 as reference)
sudo apt update && sudo apt install -y cmake g++ wget unzip

# Download and unpack sources
cd ~/Downloads
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
unzip opencv.zip

# Create build directory
mkdir -p build && cd build

# Configure
cmake  ../opencv-4.x

# Build
cmake --build .

# Install
sudo make install
```

## Remote Access to the Raspberry Pi

**How to find the Raspberry in your network**

```sh
sudo apt install nmap
ifconfig
sudo nmap 10.42.0.0/24 -v
```

### NoMachine

1. Connect the Raspberry to the same network as your PC
2. Open NoMachine
3. Connect to the device that appears when NoMachine scans your network

### SSH

```sh
ssh sdpo-ratf@10.42.0.98
> sdpo-ratf@10.42.0.98's password: 5dpo5dpo
```
