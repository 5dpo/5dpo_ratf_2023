#include "sdpo_driver_laser_2d/SdpoDriverLaser2DROS.h"

#include <exception>

#include "sdpo_driver_laser_2d/utils.h"
#include "sdpo_driver_laser_2d/RPLIDARS2.h"
#include "sdpo_driver_laser_2d/YDLIDARX4.h"

namespace sdpo_driver_laser_2d {

SdpoDriverLaser2DROS::SdpoDriverLaser2DROS() {
  try {
    readParam();
    laser_->setSerialPortParam(serial_port_name_, baud_rate_);
    laser_->setPubLaserData(
        std::bind(&SdpoDriverLaser2DROS::pubLaserData, this));
    laser_->openSerial();
  } catch (std::exception& e) {
    ROS_FATAL("[sdpo_driver_laser_2d] Error reading the node parameters (%s)",
              e.what());
    ros::shutdown();
  }

  pub_laser_ = nh.advertise<sensor_msgs::PointCloud>(
      "laser_scan_point_cloud", 1);
}

void SdpoDriverLaser2DROS::start() {
  laser_->start();
}

void SdpoDriverLaser2DROS::readParam() {
  ros::NodeHandle nh_private("~");

  if (!nh_private.hasParam("model") || !nh_private.hasParam("baud_rate")) {
    throw std::runtime_error(
        "[SdpoDriverLaser2DROS.cpp] SdpoDriverLaser2DROS::readParam: "
        "the node sdpo_driver_laser_2d_node requires the definition of the "
        "model and baud_rate parameters");
  }

  auto print_is_default_param_set =
      [&nh_private](const std::string& param_name) {
        if (!nh_private.hasParam(param_name)) {
          ROS_INFO("[sdpo_driver_laser_2d] Parameter %s not set in the "
                   "parameter server (using default value)",
                   param_name.c_str());
        }
      };

  nh_private.getParam("model", model_);
  ROS_INFO("[sdpo_driver_laser_2d] Model of the 2D laser scanner: %s",
           model_.c_str());
  if (model_ == kSdpoDriverLaser2DYDLIDARXStr) {
    laser_.reset(new YDLIDARX4());
  } else if (model_ == kSdpoDriverLaser2DRPLIDARS2Str) {
    laser_.reset(new RPLIDARS2());
  } else {
    throw std::runtime_error(
        "[SdpoDriverLaser2DROS.cpp] SdpoDriverLaser2DROS::readParam: "
        "invalid 2D laser model (check documentation for supported ones)");
  }

  print_is_default_param_set("serial_port_name");
  nh_private.param<std::string>("serial_port_name", serial_port_name_,
                                "/dev/ttyUSB0");
  ROS_INFO("[sdpo_driver_laser_2d] Serial port: %s", serial_port_name_.c_str());

  nh_private.getParam("baud_rate", baud_rate_);
  ROS_INFO("[sdpo_driver_laser_2d] Baud rate: %d bps", baud_rate_);

  print_is_default_param_set("base_frame_id");
  nh_private.param<std::string>("base_frame_id", base_frame_id_,
                                "base_footprint");
  ROS_INFO("[sdpo_driver_laser_2d] Base frame ID: %s", base_frame_id_.c_str());

  print_is_default_param_set("laser_frame_id");
  nh_private.param<std::string>("laser_frame_id", laser_frame_id_, "laser");
  ROS_INFO("[sdpo_driver_laser_2d] Laser frame ID: %s",
           laser_frame_id_.c_str());

  print_is_default_param_set("laser_pose_x");
  print_is_default_param_set("laser_pose_y");
  print_is_default_param_set("laser_pose_z");
  print_is_default_param_set("laser_pose_yaw");
  print_is_default_param_set("laser_pose_pitch");
  print_is_default_param_set("laser_pose_roll");

  nh_private.param<float>("laser_pose_x", laser_pose_x_, 0.0);
  nh_private.param<float>("laser_pose_y", laser_pose_y_, 0.0);
  nh_private.param<float>("laser_pose_z", laser_pose_z_, 0.0);
  nh_private.param<float>("laser_pose_yaw", laser_pose_yaw_, 0.0);
  nh_private.param<float>("laser_pose_pitch", laser_pose_pitch_, 0.0);
  nh_private.param<float>("laser_pose_roll", laser_pose_roll_, 0.0);

  ROS_INFO("[sdpo_driver_laser_2d] Laser > Base footprint: "
           "[%f, %f, %f] m , [%f %f %f] deg",
           laser_pose_x_, laser_pose_y_, laser_pose_z_,
           laser_pose_yaw_, laser_pose_pitch_, laser_pose_roll_);

  laser_pose_yaw_ *= M_PIf32 / 180.0f;
  laser_pose_pitch_ *= M_PIf32 / 180.0f;
  laser_pose_roll_ *= M_PIf32 / 180.0f;

  if(nh_private.hasParam("dist_min") && nh_private.hasParam("dist_max")) {
    nh_private.getParam("dist_min", dist_min_);
    nh_private.getParam("dist_max", dist_max_);
    ROS_INFO("[sdpo_driver_laser_2d] Distance range: [%f, %f] m",
             dist_min_, dist_max_);
    if (dist_max_ <= dist_min_) {
      ROS_WARN("[sdpo_driver_laser_2d] Distance range ignored "
               "(minimum must be greater than maximum)");
    } else {
      laser_->setDistRangeCheck(dist_min_, dist_max_);
    }
  } else if (nh_private.hasParam("dist_min") ||
      nh_private.hasParam("dist_max")) {
    ROS_WARN("[sdpo_driver_laser_2d] Distance range ignored "
             "(both limits must be defined)");
  } else {
    ROS_INFO("[sdpo_driver_laser_2d] Distance range not defined");
  }

  if(nh_private.hasParam("angle_min") && nh_private.hasParam("angle_max")) {
    nh_private.getParam("angle_min", ang_min_);
    nh_private.getParam("angle_max", ang_max_);
    ang_min_ = normAngRad(ang_min_ * M_PIf32 / 180.0f);
    ang_max_ = normAngRad(ang_max_ * M_PIf32 / 180.0f);
    if (ang_max_ < ang_min_) {
      std::swap(ang_min_, ang_max_);
    }
    ROS_INFO("[sdpo_driver_laser_2d] Angle range: [%f, %f] deg",
             ang_min_ * 180.0f / M_PIf32, ang_max_ * 180.0f / M_PIf32);
    laser_->setAngRangeCheck(ang_min_, ang_max_);
  } else if (nh_private.hasParam("angle_min") ||
             nh_private.hasParam("angle_max")) {
    ROS_WARN("[sdpo_driver_laser_2d] Angle range ignored "
             "(both limits must be defined)");
  } else {
    ROS_INFO("[sdpo_driver_laser_2d] Angle range not defined");
  }

  // Dynamic reconfigure
  dynamic_reconfigure::Server<sdpo_driver_laser_2d::LaserExtrinsicParamConfig>::CallbackType
      callback;
  callback = boost::bind(&SdpoDriverLaser2DROS::cfgServerCallback, this,
      _1, _2);
  cfg_server_.setCallback(callback);
}

void SdpoDriverLaser2DROS::cfgServerCallback(
    sdpo_driver_laser_2d::LaserExtrinsicParamConfig &cfg, uint32_t level) {
  laser_pose_x_ = static_cast<float>(cfg.laser_pose_x);
  laser_pose_y_ = static_cast<float>(cfg.laser_pose_y);
  laser_pose_z_ = static_cast<float>(cfg.laser_pose_z);
  laser_pose_yaw_   = static_cast<float>(cfg.laser_pose_yaw);
  laser_pose_pitch_ = static_cast<float>(cfg.laser_pose_pitch);
  laser_pose_roll_  = static_cast<float>(cfg.laser_pose_roll);

  ROS_INFO("[sdpo_driver_laser_2d] Laser > Base footprint updated: "
           "[%f, %f, %f] m , [%f %f %f] deg",
           laser_pose_x_, laser_pose_y_, laser_pose_z_,
           laser_pose_yaw_, laser_pose_pitch_, laser_pose_roll_);

  laser_pose_yaw_ *= M_PIf32 / 180.0f;
  laser_pose_pitch_ *= M_PIf32 / 180.0f;
  laser_pose_roll_ *= M_PIf32 / 180.0f;
}

void SdpoDriverLaser2DROS::pubLaserData() {
  sensor_msgs::PointCloud msg;

  msg.header.frame_id = laser_frame_id_;
  msg.header.stamp = ros::Time::now();
  msg.points.resize(laser_->data_count);
  for(size_t i = 0; i < laser_->data_count; i++) {
    msg.points.at(i).x =
        laser_->dist_data[i] * cos(laser_->ang_data[i]);
    msg.points.at(i).y =
        laser_->dist_data[i] * sin(laser_->ang_data[i]);
    msg.points.at(i).z = 0;
  }

  tf::StampedTransform laser2base_tf;
  laser2base_tf.setOrigin(tf::Vector3(
      laser_pose_x_, laser_pose_y_, laser_pose_z_));
  laser2base_tf.setRotation(tf::createQuaternionFromRPY(
      laser_pose_roll_, laser_pose_pitch_, laser_pose_yaw_));
  laser2base_tf.stamp_ = msg.header.stamp;
  laser2base_tf.frame_id_ = base_frame_id_;
  laser2base_tf.child_frame_id_ = laser_frame_id_;
  tf_broad_.sendTransform(laser2base_tf);

  pub_laser_.publish(msg);
}

} // namespace sdpo_driver_laser_2d
