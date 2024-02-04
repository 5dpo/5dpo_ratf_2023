#include "sdpo_driver_ros_camera/SdpoDriverROSCameraROS.h"

#include <exception>
#include <sstream>
#include <vector>

#include "sdpo_driver_ros_camera/ArucoArray.h"

namespace sdpo_driver_ros_camera {

SdpoDriverROSCameraROS::SdpoDriverROSCameraROS() {
  if (!readParam()) {
    ROS_FATAL("[sdpo_driver_ros_camera] Something went wrong when reading the "
              "node parameters from ROS param server");
    throw std::runtime_error("[sdpo_driver_ros_camera] Something went wrong "
        "when reading the node parameters from ROS param server");
  }
  setupCamera();

  m_pub_arucos_ =
      m_nh_.advertise<sdpo_driver_ros_camera::ArucoArray>("arucos", 1);

  m_rate.reset(new ros::Rate(m_param_pub_rate_));
}

void SdpoDriverROSCameraROS::run() {
  cv::VideoCapture cap(0);
  if (!cap.isOpened()) {
    ROS_FATAL("[sdpo_driver_ros_camera] Error openning the video stream / "
              "file (id: %s)", std::to_string(0).c_str());
    throw std::runtime_error("[sdpo_driver_ros_camera] Error openning the "
        "video stream / file (id: " + std::to_string(0) +")");
  }

  float marker_len = 0.06;
  cv::Mat obj_pts(4, 1, CV_32FC3);
  obj_pts.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_len/2.f, marker_len/2.f, 0);
  obj_pts.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_len/2.f, marker_len/2.f, 0);
  obj_pts.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_len/2.f, -marker_len/2.f, 0);
  obj_pts.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_len/2.f, -marker_len/2.f, 0);

  while (ros::ok()) {
    cv::Mat frame;
    cap >> frame;

    if (frame.empty()) {
      ROS_FATAL("[sdpo_driver_ros_camera] Empty frame");
      throw std::runtime_error("[sdpo_driver_ros_camera] Empty frame");
    }

    // - detect arucos
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    m_aruco_detector_.detectMarkers(frame, corners, ids);

    // - compute pose of marker
    std::vector<cv::Vec3d> rvecs, tvecs;
    rvecs.resize(corners.size());
    tvecs.resize(corners.size());

    if (ids.size() > 0) {
      for (int i = 0; i < corners.size(); i++) {
        solvePnP(obj_pts, corners.at(i), m_camera_mat_, m_dist_coeff_,
            rvecs.at(i), tvecs.at(i));
      }
    }

    if (m_param_debug_) {
      cv::Mat frame_copy;
      frame.copyTo(frame_copy);

      if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(frame_copy, corners, ids);
      }

      if (ids.size() > 0) {
        for (int i = 0; i < rvecs.size(); i++) {
          if (corners.at(i).size() == 4) {
            auto rvec = rvecs[i];
            auto tvec = tvecs[i];
            cv::drawFrameAxes(frame_copy, m_camera_mat_, m_dist_coeff_,
                rvec, tvec, 0.10);
          }
        }
      }

      cv::imshow("coordinate frames", frame_copy);
      cv::waitKey(1);
    }

    // - publish message
    sdpo_driver_ros_camera::ArucoArray msg;
    msg.header.frame_id = m_param_camera_frame_id_;
    msg.header.stamp = ros::Time::now();
    msg.arucos.resize(corners.size());
    for (int i = 0; i < rvecs.size(); i++) {
      msg.arucos.at(i).id = ids.at(i);
      msg.arucos.at(i).point.x = tvecs.at(i)[0];
      msg.arucos.at(i).point.y = tvecs.at(i)[1];
      msg.arucos.at(i).point.z = tvecs.at(i)[2];
    }

    tf::StampedTransform camera2base_tf;
    camera2base_tf.setOrigin(tf::Vector3(
        m_param_camera_pose_x_, m_param_camera_pose_y_,
        m_param_camera_pose_z_));
    camera2base_tf.setRotation(tf::createQuaternionFromRPY(
        m_param_camera_pose_roll_, m_param_camera_pose_pitch_,
        m_param_camera_pose_yaw_));
    camera2base_tf.stamp_ = msg.header.stamp;
    camera2base_tf.frame_id_ = m_param_base_frame_id_;
    camera2base_tf.child_frame_id_ = m_param_camera_frame_id_;
    m_tf_broad_.sendTransform(camera2base_tf);

    m_pub_arucos_.publish(msg);

    ros::spinOnce();
    m_rate->sleep();
  }

  cap.release();
  cv::destroyAllWindows();
}

bool SdpoDriverROSCameraROS::readParam() {
  ros::NodeHandle nh_priv("~");

  auto print_is_default_param_set =
      [&nh_priv](const std::string& param_name) {
    if (!nh_priv.hasParam(param_name)) {
      ROS_INFO("[sdpo_driver_ros_camera] Parameter %s not set in the "
               "parameter server (using default value)",
               param_name.c_str());
    }
  };

  print_is_default_param_set("debug");
  nh_priv.param<bool>("debug", m_param_debug_, false);
  ROS_INFO("[sdpo_driver_ros_camera] Debug enabled: %s",
           m_param_debug_? "yes" : "no");

  print_is_default_param_set("publish_rate");
  nh_priv.param<int>("publish_rate", m_param_pub_rate_, 20);
  ROS_INFO("[sdpo_driver_ros_camera] Publication rate: %d Hz",
           m_param_pub_rate_);

  print_is_default_param_set("base_frame_id");
  nh_priv.param<std::string>("base_frame_id", m_param_base_frame_id_,
                             "base_footprint");
  ROS_INFO("[sdpo_driver_ros_camera] Base frame ID: %s",
           m_param_base_frame_id_.c_str());

  print_is_default_param_set("camera_frame_id");
  nh_priv.param<std::string>("camera_frame_id", m_param_camera_frame_id_,
                             "camera");
  ROS_INFO("[sdpo_driver_ros_camera] Camera frame id: %s",
           m_param_camera_frame_id_.c_str());

  print_is_default_param_set("camera_pose_x");
  print_is_default_param_set("camera_pose_y");
  print_is_default_param_set("camera_pose_z");
  print_is_default_param_set("camera_pose_yaw");
  print_is_default_param_set("camera_pose_pitch");
  print_is_default_param_set("camera_pose_roll");

  nh_priv.param<float>("camera_pose_x", m_param_camera_pose_x_, 0.0);
  nh_priv.param<float>("camera_pose_y", m_param_camera_pose_y_, 0.0);
  nh_priv.param<float>("camera_pose_z", m_param_camera_pose_z_, 0.0);
  nh_priv.param<float>("camera_pose_yaw", m_param_camera_pose_yaw_, 0.0);
  nh_priv.param<float>("camera_pose_pitch", m_param_camera_pose_pitch_, 0.0);
  nh_priv.param<float>("camera_pose_roll", m_param_camera_pose_roll_, 0.0);

  ROS_INFO("[sdpo_driver_ros_camera] Camera > Base footprint: "
           "[%f, %f, %f] m , [%f %f %f] deg",
           m_param_camera_pose_x_, m_param_camera_pose_y_,
           m_param_camera_pose_z_, m_param_camera_pose_yaw_,
           m_param_camera_pose_pitch_, m_param_camera_pose_roll_);

  m_param_camera_pose_yaw_ *= M_PIf32 / 180.0f;
  m_param_camera_pose_pitch_ *= M_PIf32 / 180.0f;
  m_param_camera_pose_roll_ *= M_PIf32 / 180.0f;

  // Camera parameters
  if (!nh_priv.hasParam("camera_params")) {
    ROS_FATAL("[sdpo_driver_ros_camera] Path of the camera parameters file "
              "must be specified (see OpenCV intrinsic calibration tutorial)");
    return false;
  }
  nh_priv.getParam("camera_params", m_param_camera_file_path_);
  ROS_INFO("[sdpo_driver_ros_camera] Camera parameters file path: %s",
           m_param_camera_file_path_.c_str());

  cv::FileStorage fs;
  fs.open(m_param_camera_file_path_, cv::FileStorage::READ);
  fs["Camera_Matrix"] >> m_camera_mat_;
  fs["Distortion_Coefficients"] >> m_dist_coeff_;
  fs.release();

  std::stringstream str_camera_mat, str_dist_coeff;
  str_camera_mat << m_camera_mat_;
  str_dist_coeff << m_dist_coeff_;
  ROS_INFO("[sdpo_driver_ros_camera] Camera matrix:\n%s",
           str_camera_mat.str().c_str());
  ROS_INFO("[sdpo_driver_ros_camera] Distortion coefficients:\n%s",
           str_dist_coeff.str().c_str());

  // Dynamic reconfigure
  dynamic_reconfigure::Server<
      sdpo_driver_ros_camera::CameraExtrinsicParamConfig>::CallbackType
          callback;
  callback = boost::bind(&SdpoDriverROSCameraROS::cfgServerCallback, this,
      _1, _2);
  m_cfg_server_.setCallback(callback);

  return true;
}

bool SdpoDriverROSCameraROS::setupCamera() {
  cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(
      cv::aruco::DICT_5X5_100);

  m_aruco_detector_.setDictionary(dict);

  return true;
}

void SdpoDriverROSCameraROS::cfgServerCallback(
    sdpo_driver_ros_camera::CameraExtrinsicParamConfig& cfg, uint32_t level) {
  m_param_camera_pose_x_ = static_cast<float>(cfg.camera_pose_x);
  m_param_camera_pose_y_ = static_cast<float>(cfg.camera_pose_y);
  m_param_camera_pose_z_ = static_cast<float>(cfg.camera_pose_z);
  m_param_camera_pose_yaw_   = static_cast<float>(cfg.camera_pose_yaw);
  m_param_camera_pose_pitch_ = static_cast<float>(cfg.camera_pose_pitch);
  m_param_camera_pose_roll_  = static_cast<float>(cfg.camera_pose_roll);

  ROS_INFO("[sdpo_driver_ros_camera] Camera > Base footprint: "
           "[%f, %f, %f] m , [%f %f %f] deg",
           m_param_camera_pose_x_, m_param_camera_pose_y_,
           m_param_camera_pose_z_, m_param_camera_pose_yaw_,
           m_param_camera_pose_pitch_, m_param_camera_pose_roll_);

  m_param_camera_pose_yaw_ *= M_PIf32 / 180.0f;
  m_param_camera_pose_pitch_ *= M_PIf32 / 180.0f;
  m_param_camera_pose_roll_ *= M_PIf32 / 180.0f;
}

} // namespace sdpo_driver_ros_camera
