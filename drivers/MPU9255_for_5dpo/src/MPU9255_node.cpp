#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include "sensor_msgs/MagneticField.h"
#include <Eigen/Dense>

int main(int argc, char **argv){

  ros::init(argc, argv, "IMU_pub");

  ros::NodeHandle n;
  ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("imu/data_raw", 2);
  ros::Publisher pub_mag = n.advertise<sensor_msgs::MagneticField>("imu/mag", 2);


  // Get calibration matrix from parameters, matrix is a 1D vector in mode RowMajor
  std::vector<double> calib_matrix_vec,calib_matrix_vec_mag;
  if (!ros::param::get("calibration_matrix", calib_matrix_vec))
  {
      ROS_ERROR("Failed to get calibration matrix from parameter server.");
      return 1;
  }
  if (!ros::param::get("calibration_matrix_mag", calib_matrix_vec_mag))
  {
      ROS_ERROR("Failed to get calibration matrix from parameter server.");
      return 1;
  }

  Eigen::Matrix4d calib_matrix = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(calib_matrix_vec.data());
  Eigen::Matrix4d calib_matrix_mag = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(calib_matrix_vec_mag.data());


  int fd,fe;
  wiringPiSetupSys();
  fd = wiringPiI2CSetup(0x68);

        if (fd == -1) {
    printf("no i2c device found\n");
    return -1;
        }
  int16_t InBuffer[9] = {0};
  static int32_t OutBuffer[3] = {0};


   wiringPiI2CWriteReg8(fd, 0x37, 0x22); //Enable the Magnetometer
   fe = wiringPiI2CSetup(0x0C); //Start the compass
   wiringPiI2CWriteReg8(fe, 0x0A, 0x01); //Trigger first Mag reading

  while (ros::ok()){
    //http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
    //http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html
    sensor_msgs::Imu data_imu;
    sensor_msgs::MagneticField data_mag;

    data_mag.header.stamp = ros::Time::now();
    data_imu.header.stamp = data_mag.header.stamp;
    data_imu.header.frame_id = "imu_link";

    float conversion_gyro = 3.1415/(180.0*32.8f);
    float conversion_acce = 9.8/16384.0f;

    //datos acelerómetro
    InBuffer[0]=  (wiringPiI2CReadReg8 (fd, 0x3B)<<8)|wiringPiI2CReadReg8 (fd, 0x3C);
    InBuffer[1]=  (wiringPiI2CReadReg8 (fd, 0x3D)<<8)|wiringPiI2CReadReg8 (fd, 0x3E);
    InBuffer[2]=  (wiringPiI2CReadReg8 (fd, 0x3F)<<8)|wiringPiI2CReadReg8 (fd, 0x40);

    //calibrate IMU Acelerometro with the given matrix
    
    Eigen::Vector4d calibrated_ACC,raw_ACC;
    raw_ACC << InBuffer[0]*conversion_acce, InBuffer[1]*conversion_acce, InBuffer[2]*conversion_acce, 1;
    calibrated_ACC = calib_matrix * raw_ACC;

    /*
    //calibrate IMU Acelerometro with the given matrix
                      //calibrated    calib_mat      uncalibrated_________________________________
    Eigen::Vector4d calibrated_ACC = calib_matrix * Eigen::Vector4d(InBuffer[0]*conversion_acce, InBuffer[1]*conversion_acce, InBuffer[2]*conversion_acce, 1)
    */

    data_imu.linear_acceleration.x = calibrated_ACC(0);
    data_imu.linear_acceleration.y = calibrated_ACC(1);
    data_imu.linear_acceleration.z = calibrated_ACC(2);

     //datos giroscopio
    InBuffer[3]=  (wiringPiI2CReadReg8 (fd, 0x43)<<8)|wiringPiI2CReadReg8 (fd, 0x44);
    InBuffer[4]=  (wiringPiI2CReadReg8 (fd, 0x45)<<8)|wiringPiI2CReadReg8 (fd, 0x46);
    InBuffer[5]=  (wiringPiI2CReadReg8 (fd, 0x47)<<8)|wiringPiI2CReadReg8 (fd, 0x48);

    double imu_correct_fact=0.25;
    data_imu.angular_velocity.x = (InBuffer[3]*conversion_gyro-0.080399581145739)*imu_correct_fact;
    data_imu.angular_velocity.y = (InBuffer[4]*conversion_gyro-0.081997236464282)*imu_correct_fact;
    data_imu.angular_velocity.z = (InBuffer[5]*conversion_gyro-0.134425770919387)*imu_correct_fact;

    //datos magnetómetro

    uint8_t ST1;
    wiringPiI2CWriteReg8(fe, 0x0A, 0x01);

    do //wait until data has arrived
    {
     ST1=wiringPiI2CReadReg8(fe, 0x02);
    } while (!(ST1 & 0x01));

    wiringPiI2CWriteReg8(fe, 0x0A, 0x01);
    InBuffer[6]=  (wiringPiI2CReadReg8 (fe, 0x04)<<8)|wiringPiI2CReadReg8 (fe, 0x03);
    InBuffer[7]=  (wiringPiI2CReadReg8 (fe, 0x06)<<8)|wiringPiI2CReadReg8 (fe, 0x05);
    InBuffer[8]=  (wiringPiI2CReadReg8 (fe, 0x08)<<8)|wiringPiI2CReadReg8 (fe, 0x07);

    Eigen::Vector4d calibrated_MAG,raw_MAG;
    raw_MAG << InBuffer[6], InBuffer[7], InBuffer[8], 1;
    calibrated_MAG = calib_matrix_mag * raw_MAG;

    data_mag.magnetic_field.x = calibrated_MAG[0];
    data_mag.magnetic_field.y = calibrated_MAG[1];
    data_mag.magnetic_field.z = calibrated_MAG[2];

    pub_imu.publish(data_imu);

    pub_mag.publish(data_mag);

    ros::spinOnce();
    }
  return 0;
 }


