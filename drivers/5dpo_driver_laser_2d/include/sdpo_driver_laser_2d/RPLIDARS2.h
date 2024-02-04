#pragma once

#include "sdpo_driver_laser_2d/SdpoDriverLaser2D.h"

namespace sdpo_driver_laser_2d {

const size_t kLaserScanMaxNumSamplesRPLIDARS2 = 4096;
const std::string kSdpoDriverLaser2DRPLIDARS2Str = "rplidars2";

enum class RPLIDARS2State {
  kScanDescripStart1,
  kScanDescripStart2,
  kScanDescripLen1,
  kScanDescripLen2,
  kScanDescripLen3,
  kScanDescripLen4,
  kScanDescripType,
  kScanQuality,
  kScanAngleLSB,
  kScanAngleMSB,
  kScanDistLSB,
  kScanDistMSB,
  kIddle
};

class RPLIDARS2 : public SdpoDriverLaser2D {
 private:
  RPLIDARS2State state_ = RPLIDARS2State::kIddle;
  size_t sample_count_ = 0;
  uint32_t descrip_resp_len_;
  uint8_t descrip_resp_mode_;
  uint8_t descrip_resp_type_;
  bool pkg_zero_;
  uint8_t raw_sample_quality_;
  uint16_t raw_sample_angle_;
  uint16_t raw_sample_dist_;

  uint16_t raw_dist_data_[kLaserScanMaxNumSamplesRPLIDARS2];
  uint16_t raw_ang_data_[kLaserScanMaxNumSamplesRPLIDARS2];

 public:
  RPLIDARS2();
  ~RPLIDARS2() override;

  void start() override;
  void stop() override;
  void restart() override;

 protected:
  void processSerialData(unsigned char& ch) override;
  void processLaserData() override;

  void printPkgDataInfo() const override;
  void printLaserData() const override;

 private:
  inline static float rawAng2Double(const uint16_t& raw_angle) {
    return static_cast<float>(raw_angle) / 64.0f;  // >> 1 already in raw data
  }
  inline static float rawDist2Double(const uint16_t& raw_dist) {
    return static_cast<float>(raw_dist) / 4.0f;
  }
};

} // namespace sdpo_driver_laser_2d
