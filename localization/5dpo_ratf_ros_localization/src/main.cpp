#include "sdpo_ratf_ros_localization/SdpoRatfROSLocalizationROS.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sdpo_ratf_ros_localization");

  sdpo_ratf_ros_localization::SdpoRatfROSLocalizationROS ratf_localize;

  ros::spin();

  return 0;
}
