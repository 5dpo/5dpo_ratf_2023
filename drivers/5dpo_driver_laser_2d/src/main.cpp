#include "sdpo_driver_laser_2d/SdpoDriverLaser2DROS.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sdpo_driver_laser_2d");

  sdpo_driver_laser_2d::SdpoDriverLaser2DROS laser_driver;
  laser_driver.start();
  ros::spin();

  return 0;
}
