#pragma once

#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>

#include "sdpo_driver_ros_camera/CameraExtrinsicParamConfig.h"

namespace sdpo_driver_ros_camera {

class SdpoDriverROSCameraROS {
 private:
  ros::NodeHandle m_nh_;
  std::unique_ptr<ros::Rate> m_rate;

  ros::Publisher m_pub_arucos_;
  tf::TransformBroadcaster m_tf_broad_;

  dynamic_reconfigure::Server<
      sdpo_driver_ros_camera::CameraExtrinsicParamConfig> m_cfg_server_;

  bool m_param_debug_;
  int m_param_pub_rate_;
  std::string m_param_base_frame_id_;
  std::string m_param_camera_frame_id_;
  float m_param_camera_pose_x_;
  float m_param_camera_pose_y_;
  float m_param_camera_pose_z_;
  float m_param_camera_pose_yaw_;
  float m_param_camera_pose_pitch_;
  float m_param_camera_pose_roll_;

  std::string m_param_camera_file_path_;

  cv::Mat m_camera_mat_;
  cv::Mat m_dist_coeff_;

  cv::aruco::ArucoDetector m_aruco_detector_;

 public:
  SdpoDriverROSCameraROS();

  void run();

 private:
  bool readParam();
  bool setupCamera();
  void cfgServerCallback(
      sdpo_driver_ros_camera::CameraExtrinsicParamConfig& cfg, uint32_t level);
};

} // namespace sdpo_driver_ros_camera
