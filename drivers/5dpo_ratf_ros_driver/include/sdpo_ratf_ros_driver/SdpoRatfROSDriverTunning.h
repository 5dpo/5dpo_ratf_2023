#pragma once

#include <ros/ros.h>
#include <sdpo_ros_interfaces_hw/mot_data_array.h>
#include <sdpo_ros_interfaces_hw/mot_ref.h>
#include <std_srvs/SetBool.h>

#include "sdpo_ratf_ros_driver/Robot5dpoRatfTune.h"
#include "sdpo_ratf_ros_driver/SetMotorsPWM.h"

namespace sdpo_ratf_ros_driver {

class SdpoRatfROSDriverTunning {
 private:
  ros::NodeHandle nh;

  ros::Publisher pub_mot_data_;
  ros::Publisher pub_switch_;

  ros::ServiceServer srv_motors_pwm_;
  ros::ServiceServer srv_solenoid_;

  Robot5dpoRatfTune rob_;

  std::string serial_port_name_;

 public:
  SdpoRatfROSDriverTunning();
  ~SdpoRatfROSDriverTunning() = default;

  void run();

 private:
  bool readParam();

  void pubMotData();
  void pubSwitch();

  bool srvMotorsPWM(SetMotorsPWM::Request& request,
      SetMotorsPWM::Response& response);
  bool srvSolenoid(std_srvs::SetBool::Request& request,
      std_srvs::SetBool::Response& response);
};

} // namespace sdpo_ratf_ros_driver
