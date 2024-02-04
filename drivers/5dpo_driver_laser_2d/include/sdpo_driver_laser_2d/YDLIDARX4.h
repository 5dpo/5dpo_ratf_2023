#pragma once

#include "sdpo_driver_laser_2d/SdpoDriverLaser2D.h"

namespace sdpo_driver_laser_2d {

const size_t kLaserScanMaxNumSamplesYDLIDARX4 = 4096;
const std::string kSdpoDriverLaser2DYDLIDARXStr = "ydlidarx4";

enum class YDLIDARX4State {
  kPH1,
  kPH2,
  kCT,
  kLSN,
  kFSA1,
  kFSA2,
  kLSA1,
  kLSA2,
  kCS1,
  kCS2,
  kSi1,
  kSi2,
  kIddle
};

class YDLIDARX4 : public SdpoDriverLaser2D {
 private:
  YDLIDARX4State state_ = YDLIDARX4State::kIddle;
  size_t byte_count_ = 0;
  uint16_t sample_count_ = 0;
  bool pkg_zero_;
  uint16_t pkg_num_samples_;
  uint16_t pkg_check_code_;
  uint32_t raw_start_ang_;
  uint32_t raw_end_ang_;
  float start_ang_;
  float end_ang_;

  uint32_t raw_dist_data_[kLaserScanMaxNumSamplesYDLIDARX4];

 public:
  YDLIDARX4();
  ~YDLIDARX4() override;

  void start() override;
  void stop() override;
  void restart() override;

 protected:
  void processSerialData(unsigned char& ch) override;
  void processLaserData() override;

  void printPkgDataInfo() const override;
  void printLaserData() const override;

 private:
  inline static float rawStartEndAng2Double(const uint32_t& raw_angle) {
    return static_cast<float>(raw_angle) / 64.0f;  // >> 1 already in raw data
  }
  inline static float rawDist2Double(const uint32_t& raw_dist) {
    return static_cast<float>(raw_dist) / 4.0f;
  }
};

} // namespace sdpo_driver_laser_2d
