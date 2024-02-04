#pragma once

#include <array>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "sdpo_ratf_ros_localization/EKFParamConfig.h"
#include "sdpo_ratf_ros_localization/SdpoRatfEKF.h"
#include "sdpo_ratf_ros_localization/SdpoRatfBeacons.h"

namespace sdpo_ratf_ros_localization {

class SdpoRatfROSLocalizationROS {
 private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_initial_pose_;
  ros::Subscriber sub_laser_point_cloud_;
  ros::Subscriber sub_odom_;

  ros::Publisher pub_obs_beacons_;
  ros::Publisher pub_map_beacons_;
  ros::Publisher pub_pose_;

  tf::TransformBroadcaster tf_broad_;
  tf::TransformListener tf_listen_;

  dynamic_reconfigure::Server<sdpo_ratf_ros_localization::EKFParamConfig>
      cfg_server_;

  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string laser_frame_id_;

  std::vector<std::array<double, 2>> map_beacons_;
  double beacons_diam_;
  double beacons_valid_dist_;
  std::vector<SdpoRatfBeacons> beacons_;

  SdpoRatfEKF ekf_;
  double ekf_pose_ini_x_;
  double ekf_pose_ini_y_;
  double ekf_pose_ini_th_;
  double ekf_cov_ini_p_x_;
  double ekf_cov_ini_p_y_;
  double ekf_cov_ini_p_th_;
  double ekf_cov_q_d_;
  double ekf_cov_q_dn_;
  double ekf_cov_q_dth_;
  double ekf_cov_r_dist_;
  double ekf_cov_r_ang_;
  std::string ekf_mode_ini_;
  bool publish_pose_;

  bool is_initial_pose_init_ = false;
  bool is_odom_init_ = false;

  nav_msgs::Odometry odom_msg_prev_;

 public:
  SdpoRatfROSLocalizationROS();
  ~SdpoRatfROSLocalizationROS() = default;

 private:
  void readParam();

  void cfgServerCallback(sdpo_ratf_ros_localization::EKFParamConfig &cfg,
      uint32_t level);

  void subInitialPose(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void subLaserPointCloud(const sensor_msgs::PointCloud& msg);
  void subOdom(const nav_msgs::Odometry& msg);

  void pubMapBeacons();
  void pubMapTf(const std_msgs::Header& msg_header);
  void pubObsBeacons(const std_msgs::Header& msg_header);
  void pubPose(const std_msgs::Header& msg_header);

  void detectBeacons(const sensor_msgs::PointCloud& msg);
};

} // namespace sdpo_ratf_ros_localization
