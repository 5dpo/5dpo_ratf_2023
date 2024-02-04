#include "sdpo_ratf_ros_localization/SdpoRatfROSLocalizationROS.h"

#include <exception>
#include <mutex>
#include <sstream>

#include <XmlRpcException.h>

#include "sdpo_ratf_ros_localization/utils.h"

namespace sdpo_ratf_ros_localization {

SdpoRatfROSLocalizationROS::SdpoRatfROSLocalizationROS() {
  try {
    readParam();
  } catch (std::exception& e) {
    ROS_FATAL("[sdpo_ratf_ros_localization] Error reading the node parameters "
              "(%s)", e.what());
    ros::shutdown();
  }

  ekf_.initCovP(ekf_cov_ini_p_x_, ekf_cov_ini_p_y_, ekf_cov_ini_p_th_);
  ekf_.initCovQ(ekf_cov_q_d_    , ekf_cov_q_dn_   , ekf_cov_q_dth_);
  ekf_.initCovR(ekf_cov_r_dist_ , ekf_cov_r_ang_);
  ekf_.initPose(ekf_pose_ini_x_ , ekf_pose_ini_y_ , ekf_pose_ini_th_);

  sub_initial_pose_ = nh_.subscribe("initial_pose", 5,
      &SdpoRatfROSLocalizationROS::subInitialPose, this);
  sub_laser_point_cloud_ = nh_.subscribe("laser_scan_point_cloud", 5,
      &SdpoRatfROSLocalizationROS::subLaserPointCloud, this);
  sub_odom_ = nh_.subscribe("odom", 10,
      &SdpoRatfROSLocalizationROS::subOdom, this);
  
  pub_obs_beacons_ = nh_.advertise<sensor_msgs::PointCloud>("beacons", 5);
  pub_map_beacons_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "map_beacons", 5, true);
  if (publish_pose_) {
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 5);
  }

  pubMapBeacons();

  // Dynamic reconfigure
  dynamic_reconfigure::Server<sdpo_ratf_ros_localization::EKFParamConfig>::CallbackType
      callback;
  callback = boost::bind(&SdpoRatfROSLocalizationROS::cfgServerCallback, this,
      _1, _2);
  cfg_server_.setCallback(callback);
}

void SdpoRatfROSLocalizationROS::readParam() {
  ros::NodeHandle nh_private("~");

  auto print_is_default_param_set =
      [&nh_private](const std::string& param_name) {
        if (!nh_private.hasParam(param_name)) {
          ROS_INFO("[sdpo_ratf_ros_localization] Parameter %s not set in the "
                   "parameter server (using default value)",
                   param_name.c_str());
        }
      };

  // Coordinate Frame IDs
  print_is_default_param_set("map_frame_id");
  nh_private.param<std::string>("map_frame_id", map_frame_id_,
                                "map");
  ROS_INFO("[sdpo_ratf_ros_localization] Map frame ID: %s",
           map_frame_id_.c_str());

  print_is_default_param_set("odom_frame_id");
  nh_private.param<std::string>("odom_frame_id", odom_frame_id_,
                                "odom");
  ROS_INFO("[sdpo_ratf_ros_localization] Odometry frame ID: %s",
           odom_frame_id_.c_str());

  print_is_default_param_set("base_frame_id");
  nh_private.param<std::string>("base_frame_id", base_frame_id_,
                                "base_footprint");
  ROS_INFO("[sdpo_ratf_ros_localization] Base footprint frame ID: %s",
           base_frame_id_.c_str());

  print_is_default_param_set("laser_frame_id");
  nh_private.param<std::string>("laser_frame_id", laser_frame_id_,
                                "laser");
  ROS_INFO("[sdpo_ratf_ros_localization] Laser frame ID: %s",
           laser_frame_id_.c_str());


  // Map Beacons
  print_is_default_param_set("beacons_diam");
  if(!nh_private.getParam("beacons_diam", beacons_diam_)) {
    beacons_diam_ = 0.09;
  }
  ROS_INFO("[sdpo_ratf_ros_localization] Beacons diameters: %lf",
      beacons_diam_);

  print_is_default_param_set("beacons_valid_dist");
  if(!nh_private.getParam("beacons_valid_dist", beacons_valid_dist_)) {
    beacons_valid_dist_ = 0.20;
  }
  ROS_INFO("[sdpo_ratf_ros_localization] Beacons valid distance: %lf m",
           beacons_valid_dist_);

  XmlRpc::XmlRpcValue beacons;
  try {   // always define matrix with decimal points (can cause problems)
    nh_private.getParam("beacons", beacons);
    ROS_ASSERT(beacons.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT((beacons.size() % 2) == 0);

    map_beacons_.resize(static_cast<size_t>(beacons.size() / 2));
    int i = 0;
    for (auto& beacon : map_beacons_) {
      for (int j = 0; j < 2; j++) {
        try { // Source: https://github.com/cra-ros-pkg/robot_localization/blob/32896d6d1aaec5a92c3a65e5f16dfc0b859a7d26/src/ros_filter.cpp#L1556
          std::ostringstream ostr;
          ostr << beacons[i*2 + j];
          beacon[j] = std::stof(ostr.str());

        } catch (XmlRpc::XmlRpcException& e) {
          throw e;
        } catch (...) {
          throw;
        }
      }
      i++;
    }

    std::ostringstream ostr;
    for (auto& beacon : map_beacons_) {
      ostr << std::endl << "  x: " << beacon[0] << " , y: " << beacon[1];
    }

    beacons_.resize(map_beacons_.size());

    ROS_INFO("[sdpo_ratf_localization_ros] Beacons map (#: %lu):%s",
             map_beacons_.size(), ostr.str().c_str());

  } catch (XmlRpc::XmlRpcException& e) {
    throw std::runtime_error(e.getMessage());
  } catch (...) {
    throw std::runtime_error(
        "[SdpoRatfROSLocalizationROS.cpp] "
        "SdpoRatfROSLocalizationROS::readParam: "
        "something went wrong when reading beacons from the ROS param server");
  }

  // Extended Kalman Filter (EKF) Parameters
  print_is_default_param_set("ekf_pose_ini_x");
  print_is_default_param_set("ekf_pose_ini_y");
  print_is_default_param_set("ekf_pose_ini_th");
  if(!nh_private.getParam("ekf_pose_ini_x", ekf_pose_ini_x_)) {
    ekf_pose_ini_x_ = 0.0;
  }
  if(!nh_private.getParam("ekf_pose_ini_y", ekf_pose_ini_y_)) {
    ekf_pose_ini_y_ = 0.0;
  }
  if(!nh_private.getParam("ekf_pose_ini_th", ekf_pose_ini_th_)) {
    ekf_pose_ini_th_ = 0.0;
  }
  ekf_pose_ini_th_ = deg2rad(ekf_pose_ini_th_);
  ROS_INFO("[sdpo_ratf_localization_ros] EKF Initial Pose: "
           "[ %lf m , %lf m , %lf deg ]",
           ekf_pose_ini_x_, ekf_pose_ini_y_, rad2deg(ekf_pose_ini_th_));

  print_is_default_param_set("ekf_cov_ini_p_x");
  print_is_default_param_set("ekf_cov_ini_p_y");
  print_is_default_param_set("ekf_cov_ini_p_th");
  if(!nh_private.getParam("ekf_cov_ini_p_x", ekf_cov_ini_p_x_)) {
    ekf_cov_ini_p_x_ = 0.1;
  }
  if(!nh_private.getParam("ekf_cov_ini_p_y", ekf_cov_ini_p_y_)) {
    ekf_cov_ini_p_y_ = 0.1;
  }
  if(!nh_private.getParam("ekf_cov_ini_p_th", ekf_cov_ini_p_th_)) {
    ekf_cov_ini_p_th_ = 0.1;
  }
  ROS_INFO("[sdpo_ratf_localization_ros] EKF Initial Covariance P: "
           "diag{%lf %lf %lf}",
           ekf_cov_ini_p_x_, ekf_cov_ini_p_y_, ekf_cov_ini_p_th_);

  print_is_default_param_set("ekf_cov_q_d");
  print_is_default_param_set("ekf_cov_q_dn");
  print_is_default_param_set("ekf_cov_q_dth");
  if(!nh_private.getParam("ekf_cov_q_d", ekf_cov_q_d_)) {
    ekf_cov_q_d_ = 0.1;
  }
  if(!nh_private.getParam("ekf_cov_q_dn", ekf_cov_q_dn_)) {
    ekf_cov_q_dn_ = 0.1;
  }
  if(!nh_private.getParam("ekf_cov_q_dth", ekf_cov_q_dth_)) {
    ekf_cov_q_dth_ = 0.05;
  }
  ROS_INFO("[sdpo_ratf_localization_ros] EKF Covariance Q: diag{%lf %lf %lf}",
           ekf_cov_q_d_, ekf_cov_q_dn_, ekf_cov_q_dth_);

  print_is_default_param_set("ekf_cov_r_dist");
  print_is_default_param_set("ekf_cov_r_ang");
  if(!nh_private.getParam("ekf_cov_r_dist", ekf_cov_r_dist_)) {
    ekf_cov_r_dist_ = 0.0001;
  }
  if(!nh_private.getParam("ekf_cov_r_ang", ekf_cov_r_ang_)) {
    ekf_cov_r_ang_ = 0.001;
  }
  ROS_INFO("[sdpo_ratf_localization_ros] EKF Covariance R: diag{%lf %lf}",
           ekf_cov_r_dist_, ekf_cov_r_ang_);

  print_is_default_param_set("ekf_mode_ini");
  nh_private.param<std::string>("ekf_mode_ini", ekf_mode_ini_,
      kSdpoRatfEKFModeFusionStr);
  ekf_.setMode(ekf_mode_ini_);
  ROS_INFO("[sdpo_ratf_localization_ros] EKF Mode: %s",
           ekf_.getModeStr().c_str());

  print_is_default_param_set("publish_pose");
  nh_private.param<bool>("publish_pose", publish_pose_, false);
  ROS_INFO("[sdpo_ratf_localization_ros] Publish pose: %s",
           publish_pose_? "yes" : "no");
}

void SdpoRatfROSLocalizationROS::cfgServerCallback(
    sdpo_ratf_ros_localization::EKFParamConfig &cfg, uint32_t level) {
  ekf_cov_q_d_ = cfg.ekf_cov_q_d;
  ekf_cov_q_dn_ = cfg.ekf_cov_q_dn;
  ekf_cov_q_dth_ = cfg.ekf_cov_q_dth;
  ROS_INFO("[sdpo_ratf_localization_ros] EKF Covariance Q updated: "
           "diag{%lf %lf %lf}", ekf_cov_q_d_, ekf_cov_q_dn_, ekf_cov_q_dth_);

  ekf_cov_r_dist_ = cfg.ekf_cov_r_dist;
  ekf_cov_r_ang_ = cfg.ekf_cov_r_ang;
  ROS_INFO("[sdpo_ratf_localization_ros] EKF Covariance R updated: "
           "diag{%lf %lf}", ekf_cov_r_dist_, ekf_cov_r_ang_);

  ekf_.setMode(cfg.ekf_mode_ini);
  ROS_INFO("[sdpo_ratf_localization_ros] EKF Mode updated: %s",
           ekf_.getModeStr().c_str());
}

void SdpoRatfROSLocalizationROS::subInitialPose(
    const geometry_msgs::PoseWithCovarianceStamped& msg) {
  if (msg.header.frame_id != map_frame_id_) {
    ROS_WARN("[sdpo_ratf_localization_ros] Frame ID of initial pose wrong "
             "(expected id: %s vs %s)",
             map_frame_id_.c_str(), msg.header.frame_id.c_str());
    return;
  }

  ekf_.initCovP(ekf_cov_ini_p_x_, ekf_cov_ini_p_y_, ekf_cov_ini_p_th_);
  ekf_.initPose(msg.pose.pose.position.x, msg.pose.pose.position.y,
      tf::getYaw(msg.pose.pose.orientation));

  try{
    pubMapTf(msg.header);
  } catch (std::exception& e) {
    ROS_ERROR("[sdpo_ratf_localization_ros] Error when setting initial pose of "
              "the EKF (%s)", e.what());
    return;
  }

  ROS_INFO("[sdpo_ratf_localization_ros] New initial pose set in EKF "
           "(%lf m, %lf m, %lf deg)",
           ekf_.XR(0), ekf_.XR(1), rad2deg(ekf_.XR(2)));

  is_initial_pose_init_ = true;
}

void SdpoRatfROSLocalizationROS::subLaserPointCloud(
    const sensor_msgs::PointCloud& msg) {
  if (is_initial_pose_init_) {
    try{
      detectBeacons(msg);
    } catch (std::exception& e) {
      ROS_ERROR("[sdpo_ratf_localization_ros] Error when processing the laser "
                "point cloud (%s)", e.what());
      return;
    }

    bool do_tf_update = false;
    for (size_t i = 0; i < beacons_.size(); i++) {
      if (beacons_[i].num_pts > 0) {
        do_tf_update = true;
        ekf_.update(beacons_[i], map_beacons_[i]);
      }
    }

    if (do_tf_update) {
      try{
        pubMapTf(msg.header);
        pubPose(msg.header);
      } catch (std::exception& e) {
        ROS_ERROR("[sdpo_ratf_localization_ros] Error when updating the pose of"
                  " the EKF (%s)", e.what());
        return;
      }
    }

    pubObsBeacons(msg.header);
  }
}

void SdpoRatfROSLocalizationROS::subOdom(const nav_msgs::Odometry& msg) {
  if (!is_odom_init_) {
    is_odom_init_ = true;
  } else {
    double dx, dy, dth;
    double dd, ddn;
    double th;

    dx  = msg.pose.pose.position.x - odom_msg_prev_.pose.pose.position.x;
    dy  = msg.pose.pose.position.y - odom_msg_prev_.pose.pose.position.y;
    dth = normAngRad(tf::getYaw(msg.pose.pose.orientation) -
                     tf::getYaw(odom_msg_prev_.pose.pose.orientation));

    th = normAngRad(tf::getYaw(odom_msg_prev_.pose.pose.orientation));

    dd  =  dx * cos(th) + dy * sin(th);
    ddn = -dx * sin(th) + dy * cos(th);

    ekf_.predict(dd, ddn, dth);
  }

  try{
    pubMapTf(msg.header);
    pubPose(msg.header);
  } catch (std::exception& e) {
    ROS_ERROR("[sdpo_ratf_localization_ros] Error when updating the TF with the"
              " prediction step of the EKF (%s)", e.what());
    is_odom_init_ = false;
    return;
  }

  odom_msg_prev_ = msg;

  is_initial_pose_init_ = true;
}

void SdpoRatfROSLocalizationROS::pubMapBeacons() {
  ros::Time stamp = ros::Time::now();
  visualization_msgs::MarkerArray map;
  map.markers.resize(map_beacons_.size());

  for (size_t i = 0; i < map_beacons_.size(); i++) {
    map.markers[i].header.frame_id = map_frame_id_;
    map.markers[i].header.stamp = stamp;
    map.markers[i].ns = "map";
    map.markers[i].id = i;
    map.markers[i].type = visualization_msgs::Marker::CYLINDER;
    map.markers[i].action = visualization_msgs::Marker::ADD;
    map.markers[i].pose.position.x = map_beacons_[i][0];
    map.markers[i].pose.position.y = map_beacons_[i][1];
    map.markers[i].pose.position.z = 0.0;
    map.markers[i].pose.orientation.x = 0.0;
    map.markers[i].pose.orientation.y = 0.0;
    map.markers[i].pose.orientation.z = 0.0;
    map.markers[i].pose.orientation.w = 1.0;
    map.markers[i].scale.x = beacons_diam_;
    map.markers[i].scale.y = beacons_diam_;
    map.markers[i].scale.z = 0.50;
    map.markers[i].color.a = 0.25;
    map.markers[i].color.r = 0.5;
    map.markers[i].color.g = 0.5;
    map.markers[i].color.b = 0.5;
  }

  pub_map_beacons_.publish(map);
}

void SdpoRatfROSLocalizationROS::pubMapTf(const std_msgs::Header& msg_header) {
  tf::StampedTransform tf_base2odom;
  try {
    tf_listen_.lookupTransform(odom_frame_id_, base_frame_id_, ros::Time(0),
        tf_base2odom);
  } catch (tf::TransformException& e) {
    throw std::runtime_error(e.what());
  }

  tf::StampedTransform tf_base2map, tf_odom2map;
  tf_base2map.setOrigin(tf::Vector3(ekf_.XR(0), ekf_.XR(1), 0.0));
  tf_base2map.setRotation(tf::createQuaternionFromYaw(ekf_.XR(2)));
  tf::Transform result = tf_base2map * tf_base2odom.inverse();

  tf_odom2map.setOrigin(result.getOrigin());
  tf_odom2map.setRotation(result.getRotation());
  tf_odom2map.stamp_ = msg_header.stamp;
  tf_odom2map.frame_id_ = map_frame_id_;
  tf_odom2map.child_frame_id_ = odom_frame_id_;
  
  tf_broad_.sendTransform(tf_odom2map);
}

void SdpoRatfROSLocalizationROS::pubObsBeacons(
    const std_msgs::Header& msg_header) {
  sensor_msgs::PointCloud msg;
  msg.header = msg_header;
  msg.header.frame_id = map_frame_id_;

  for (auto& beacon : beacons_) {
    if (beacon.num_pts > 0) {
      geometry_msgs::Point32 pt;
      pt.x = static_cast<float>(beacon.x);
      pt.y = static_cast<float>(beacon.y);
      pt.z = 0.0f;
      msg.points.push_back(pt);
    }
  }

  if (!msg.points.empty()) {
    pub_obs_beacons_.publish(msg);
  }
}

void SdpoRatfROSLocalizationROS::pubPose(const std_msgs::Header& msg_header) {
  if (publish_pose_) {
    geometry_msgs::PoseStamped msg;
    msg.header = msg_header;
    msg.header.frame_id = map_frame_id_;

    msg.pose.position.x = ekf_.XR(0);
    msg.pose.position.y = ekf_.XR(1);
    msg.pose.position.z = 0.0;

    tf::Quaternion tf_q = tf::createQuaternionFromYaw(ekf_.XR(2));
    msg.pose.orientation.w = tf_q.getW();
    msg.pose.orientation.x = tf_q.getX();
    msg.pose.orientation.y = tf_q.getY();
    msg.pose.orientation.z = tf_q.getZ();

    pub_pose_.publish(msg);
  }
}

void SdpoRatfROSLocalizationROS::detectBeacons(
    const sensor_msgs::PointCloud& msg) {
  tf::StampedTransform tf_laser2map;
  try {
    tf_listen_.lookupTransform(map_frame_id_, laser_frame_id_, ros::Time(0),
        tf_laser2map);
  } catch (tf::TransformException& e) {
    throw std::runtime_error(e.what());
  }

  for (auto& beacon : beacons_) {
    beacon.x = 0;
    beacon.y = 0;
    beacon.num_pts = 0;
  }

  for (size_t i = 0; i < msg.points.size(); i++) {
    double pt_x = static_cast<double>(msg.points[i].x);
    double pt_y = static_cast<double>(msg.points[i].y);

    double pt_dist = dist(pt_x, pt_y);
    double pt_ang = atan2(pt_y, pt_x);
    double meas_dist = pt_dist + beacons_diam_ / 2.0;

    tf::Vector3 meas_pt(meas_dist*cos(pt_ang), meas_dist*sin(pt_ang), 0.0);
    tf::Vector3 meas_map_pt = tf_laser2map * meas_pt;

    for(size_t j = 0; j < map_beacons_.size(); j++) {
      if (dist(map_beacons_[j][0]-meas_map_pt[0],
               map_beacons_[j][1]-meas_map_pt[1] ) < beacons_valid_dist_) {
        beacons_[j].num_pts++;
        beacons_[j].x =
            ((beacons_[j].x * (beacons_[j].num_pts - 1.0)) + meas_map_pt[0])/
            beacons_[j].num_pts;
        beacons_[j].y =
            ((beacons_[j].y * (beacons_[j].num_pts - 1.0)) + meas_map_pt[1])/
            beacons_[j].num_pts;
      }
    }
  }

  for (auto& beacon : beacons_) {
    beacon.dist = dist(beacon.x - ekf_.XR(0), beacon.y - ekf_.XR(1));
    beacon.ang = normAngRad(
        atan2(beacon.y - ekf_.XR(1), beacon.x - ekf_.XR(0)) -
        ekf_.XR(2));
  }
}

} // namespace sdpo_ratf_ros_localization
