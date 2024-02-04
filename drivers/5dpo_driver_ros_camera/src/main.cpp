#include "sdpo_driver_ros_camera/SdpoDriverROSCameraROS.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sdpo_driver_ros_camera");

  try {
    sdpo_driver_ros_camera::SdpoDriverROSCameraROS aruco_detector;

    aruco_detector.run();

  } catch (...) {
    return -1;
  }

  return 0;
}