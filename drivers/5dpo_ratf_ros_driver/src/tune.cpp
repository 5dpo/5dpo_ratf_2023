#include "sdpo_ratf_ros_driver/SdpoRatfROSDriverTunning.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sdpo_ratf_ros_driver");

  sdpo_ratf_ros_driver::SdpoRatfROSDriverTunning ratf_ros_driver;
  ros::spin();

  return 0;
}
