#include "markers.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sdpo_ratf_ros_gui");

  sdpo_ratf_ros_gui::boxMarkers boxes;

  ros::spin();

  return 0;
}