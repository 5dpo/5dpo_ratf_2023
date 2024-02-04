#pragma once

#include <iostream>

namespace sdpo_ratf_ros_localization {

struct SdpoRatfBeacons {
 public:
  int num_pts;
  double x;
  double y;
  double dist;
  double ang;
};

} // namespace sdpo_ratf_ros_localization
