#include "sdpo_ratf_ros_driver/SdpoRatfROSDriverTunning.h"

#include <std_msgs/Bool.h>

namespace sdpo_ratf_ros_driver {

SdpoRatfROSDriverTunning::SdpoRatfROSDriverTunning() {
  readParam();

  pub_mot_data_ = nh.advertise<sdpo_ros_interfaces_hw::mot_data_array>(
      "motors_data", 1);
  pub_switch_ = nh.advertise<std_msgs::Bool>("switch_state", 1);

  rob_.setSerialPortName(serial_port_name_);
  rob_.openSerial();
  rob_.run = std::bind(&SdpoRatfROSDriverTunning::run, this);
  rob_.init();

  srv_motors_pwm_ = nh.advertiseService("set_motors_pwm",
      &SdpoRatfROSDriverTunning::srvMotorsPWM, this);
  srv_solenoid_ = nh.advertiseService("set_solenoid_state",
      &SdpoRatfROSDriverTunning::srvSolenoid, this);
}

void SdpoRatfROSDriverTunning::run() {
  pubMotData();
  pubSwitch();
}

bool SdpoRatfROSDriverTunning::readParam() {
  ros::NodeHandle nh_private("~");

  for (auto& m : rob_.mot) {
    nh_private.param<double>("encoder_res", m.encoder_res, 48.0);
    nh_private.param<double>("gear_reduction", m.gear_reduction, 64.0);
  }
  nh_private.param<std::string>("serial_port_name", serial_port_name_,
                                "/dev/ttyACM0");

  auto print_is_default_param_set =
      [&nh_private](const std::string& param_name) {
    if (!nh_private.hasParam(param_name)) {
      ROS_INFO("[sdpo_ratf_ros_driver_tune] Parameter %s not set in the "
               "parameter server (using default value)",
               param_name.c_str());
    }
  };

  print_is_default_param_set("encoder_res");
  ROS_INFO("[sdpo_ratf_ros_driver_tune] Encoder resolution: %lf (ticks/rev)",
           rob_.mot[0].encoder_res);

  print_is_default_param_set("gear_reduction");
  ROS_INFO("[sdpo_ratf_ros_driver_tune] Gear reduction ratio: %lf (n:1)",
           rob_.mot[0].gear_reduction);

  print_is_default_param_set("serial_port_name");
  ROS_INFO("[sdpo_ratf_ros_driver_tune] Serial port: %s",
           serial_port_name_.c_str());

  return true;
}

void SdpoRatfROSDriverTunning::pubMotData() {
  sdpo_ros_interfaces_hw::mot_data_array msg;

  msg.stamp = ros::Time::now();
  msg.mot_data_array.resize(4);

  rob_.mtx_.lock();
  for (int i = 0; i < 4; i++) {
    msg.mot_data_array[i].sample_period = rob_.mot[i].sample_time;
    msg.mot_data_array[i].pwm = rob_.mot[i].pwm;
    msg.mot_data_array[i].encoder_delta = rob_.mot[i].getEncTicksDeltaPub();
    msg.mot_data_array[i].ticks_per_rev =
        rob_.mot[i].encoder_res * rob_.mot[i].gear_reduction;
    msg.mot_data_array[i].angular_speed = rob_.mot[i].w;
  }
  rob_.mtx_.unlock();

  pub_mot_data_.publish(msg);
}

void SdpoRatfROSDriverTunning::pubSwitch() {
  std_msgs::Bool msg;

  rob_.mtx_.lock();
  msg.data = rob_.switch_state;
  rob_.mtx_.unlock();

  pub_switch_.publish(msg);
}

bool SdpoRatfROSDriverTunning::srvMotorsPWM(SetMotorsPWM::Request& request,
    SetMotorsPWM::Response& response) {
  if (request.motors_pwm.size() != sizeof(rob_.mot)/sizeof(Motor)) {
    ROS_WARN("[sdpo_ratf_ros_driver_tune] Expected to receive PWM for 4 motors "
             "instead of only %ld. Command ignored...",
             request.motors_pwm.size());

    return false;
  }

  rob_.mtx_.lock();
  for (size_t i = 0; i < request.motors_pwm.size(); i++) {
    rob_.mot[i].setPWM(request.motors_pwm[i]);
  }
  ROS_INFO("[sdpo_ratf_ros_driver_tune] PWM set: [%d %d %d %d]",
           rob_.mot[0].pwm, rob_.mot[1].pwm, rob_.mot[2].pwm, rob_.mot[3].pwm);
  rob_.mtx_.unlock();

  return true;
}

bool SdpoRatfROSDriverTunning::srvSolenoid(std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  rob_.mtx_.lock();
  rob_.solenoid_state = request.data;
  rob_.mtx_.unlock();

  response.success = true;
  response.message = "";
  return true;
}

} // namespace sdpo_ratf_ros_driver
