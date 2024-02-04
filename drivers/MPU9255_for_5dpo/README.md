ROS MPU9255 Node
================

c++ ROS node wrapper for the mpu9255 gyroscope / accelerometer/magnetometer.\
Reads accelerometer, gyroscopic and magnetic data in 3D giving 9 Degrees of Freedom.\
Publishes sensor_msgs::IMU to /imu/data_raw topic(angular_velocity and linear_acceleration).\
Publishes sensor_msgs::MagneticField to /imu/data_raw topic(angular_velocity and linear_acceleration).\
Supported interface : I2C.\
used in Raspberry PI 3.\

Installation
------------

First install WiringPI:
	
	cd
	git clone git://git.drogon.net/wiringPi
	cd ~/wiringPi
	./build

Then clone this repository into your ROS workspace(src folder):

    git clone https://github.com/mdleiton/MPU9255.git
    
Compile it:

    catkin_make
	g++ offsetIMU.cpp -lwiringPi -o offsetIMU

Run a node:

	./offsetIMU ofssetData.txt
	source devel/setup.bash
	rosrun MPU9255 MPU9255_node

Review the published data with:

    rostopic echo /imu/data_raw
	rostopic echo /imu/mag


Complementary filter
--------------------

install imu-tools:

	sudo apt-get install ros-kinetic-imu-tools

run launch file:

	source devel/setup.bash
	roslaunch MPU9255 imu.launch

reference: http://wiki.ros.org/imu_complementary_filter

Create a rosbag file (.bag)
---------------------------

run launch file:

	source devel/setup.bash
	roslaunch MPU9255 record_topic.launch

Get data from the topics of the .bag file:

	rostopic echo -b file.bag -p /topic

Visualize Rviz:
---------------

	source devel/setup.bash
	roslaunch MPU9255 record_topic.launch
	rviz 				# other terminal

Configure as FIG.1 or load config file in (rviz_config/imu.rviz)

![Alt text](media/fig1.png "RVIZ configuration")

GIF:
![Alt text](media/imu.gif "RVIZ working with imu")
		


More information about published topics:

	http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
	http://docs.ros.org/jade/api/sensor_msgs/html/msg/MagneticField.html

