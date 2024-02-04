#include "sdpo_driver_laser_2d/RPLIDARS2.h"

#include "sdpo_driver_laser_2d/utils.h"

namespace sdpo_driver_laser_2d {

RPLIDARS2::RPLIDARS2() {
  dist_data.resize(kLaserScanMaxNumSamplesRPLIDARS2);
  ang_data.resize(kLaserScanMaxNumSamplesRPLIDARS2);
}

RPLIDARS2::~RPLIDARS2() {
  if (isSerialOpen()) {
    stop();
  }
  closeSerial();
}

void RPLIDARS2::start() {
  char start_scan_cmd[2];
  start_scan_cmd[0] = (char) 0xA5;
  start_scan_cmd[1] = (char) 0x20;

  serial_async_->write(start_scan_cmd, 2);
}

void RPLIDARS2::stop() {
  char stop_scan_cmd[2];
  stop_scan_cmd[0] = (char) 0xA5;
  stop_scan_cmd[1] = (char) 0x25;

  serial_async_->write(stop_scan_cmd, 2);
}

void RPLIDARS2::restart() {
  char restart_scan_cmd[2];
  restart_scan_cmd[0] = (char) 0xA5;
  restart_scan_cmd[1] = (char) 0x40;

  serial_async_->write(restart_scan_cmd, 2);
}

void RPLIDARS2::processSerialData(unsigned char& ch) {
  // Determine the type of data of the current byte (state transitions)
  switch (state_) {
  // - standard scan
  //   - scan initial descriptor (standard scan)
  case RPLIDARS2State::kScanDescripStart1:
    if (ch == 0x5A) {
      state_ = RPLIDARS2State::kScanDescripStart2;
    } else {
      state_ = RPLIDARS2State::kIddle;
    }
    break;
  case RPLIDARS2State::kScanDescripStart2:
    state_ = RPLIDARS2State::kScanDescripLen1;
    break;
  case RPLIDARS2State::kScanDescripLen1:
    state_ = RPLIDARS2State::kScanDescripLen2;
    break;
  case RPLIDARS2State::kScanDescripLen2:
    state_ = RPLIDARS2State::kScanDescripLen3;
    break;
  case RPLIDARS2State::kScanDescripLen3:
    state_ = RPLIDARS2State::kScanDescripLen4;
    break;
  case RPLIDARS2State::kScanDescripLen4:
    state_ = RPLIDARS2State::kScanDescripType;
    break;
  case RPLIDARS2State::kScanDescripType:
    state_ = RPLIDARS2State::kScanQuality;
    break;
  //   - scan data (standard scan)
  case RPLIDARS2State::kScanQuality:
    state_ = RPLIDARS2State::kScanAngleLSB;
    break;
  case RPLIDARS2State::kScanAngleLSB:
    state_ = RPLIDARS2State::kScanAngleMSB;
    break;
  case RPLIDARS2State::kScanAngleMSB:
    state_ = RPLIDARS2State::kScanDistLSB;
    break;
  case RPLIDARS2State::kScanDistLSB:
    state_ = RPLIDARS2State::kScanDistMSB;
    break;
  case RPLIDARS2State::kScanDistMSB:
    state_ = RPLIDARS2State::kScanQuality;
    break;
  // - iddle
  case RPLIDARS2State::kIddle:
    if (ch == 0xA5) {
      state_ = RPLIDARS2State::kScanDescripStart1;
    }
    break;
  }

  // Process the current data byte (state actions)
  switch (state_) {
  // - standard scan
  //   - scan initial descriptor (standard scan)
  case RPLIDARS2State::kScanDescripStart1:
  case RPLIDARS2State::kScanDescripStart2:
    break;
  case RPLIDARS2State::kScanDescripLen1:
    descrip_resp_len_ = (ch & 0x000000FF);
    break;
  case RPLIDARS2State::kScanDescripLen2:
    descrip_resp_len_ = (descrip_resp_len_ | ((ch & 0x000000FF) << 8));
    break;
  case RPLIDARS2State::kScanDescripLen3:
    descrip_resp_len_ = (descrip_resp_len_ | ((ch & 0x000000FF) << 16));
    break;
  case RPLIDARS2State::kScanDescripLen4:
    descrip_resp_len_ = (descrip_resp_len_ | ((ch & 0x0000003F) << 24));
    descrip_resp_mode_ = (ch & 0xC0);
    break;
  case RPLIDARS2State::kScanDescripType:
    descrip_resp_type_ = ch;
    break;
  //   - scan data (standard scan)
  case RPLIDARS2State::kScanQuality:
    if ((ch & 0x01) == 0x01) {
      pkg_zero_ = true;
      processLaserData();
      //printPkgDataInfo();
      if (pubLaserData) {
        pubLaserData();
      }
      data_count = 0;
      sample_count_ = 0;
    } else {
      pkg_zero_ = false;
    }
    raw_sample_quality_ = ((ch & 0xFC) >> 2);
    break;
  case RPLIDARS2State::kScanAngleLSB:
    raw_sample_angle_ = (ch & 0x00FF);
    break;
  case RPLIDARS2State::kScanAngleMSB:
    raw_sample_angle_ = ((raw_sample_angle_ | (ch << 8)) >> 1);
    raw_ang_data_[sample_count_] = raw_sample_angle_;
    break;
  case RPLIDARS2State::kScanDistLSB:
    raw_sample_dist_ = (ch & 0x00FF);
    break;
  case RPLIDARS2State::kScanDistMSB:
    raw_sample_dist_ = (raw_sample_dist_ | (ch << 8));
    raw_dist_data_[sample_count_] = raw_sample_dist_;
    sample_count_++;
    break;
  // - iddle
  case RPLIDARS2State::kIddle:
    data_count = 0;
    sample_count_ = 0;
    break;
  }
}

void RPLIDARS2::processLaserData() {
  if (sample_count_ > 0) {
    bool is_sample_ok;

    for(size_t i = 0; i < sample_count_; i++) {
      // distance data (mm)
      dist_data[data_count] = rawDist2Double(raw_dist_data_[i]) / 1000.0f;

      // angle data (deg)
      ang_data[data_count] = rawAng2Double(raw_ang_data_[i]);

      // angle data (rad) (CW > CCW)
      ang_data[data_count] =
          -normAngRad(ang_data[data_count] * M_PIf32 / 180.0f);

      // check distance and angle ranges variable
      is_sample_ok = true;
      if (dist_range_check_) {
        if ((dist_data[data_count] < dist_min_) ||
            (dist_data[data_count] > dist_max_)) {
          is_sample_ok = false;
        }
      }
      if (ang_range_check_) {
        if ((ang_data[data_count] < ang_min_) ||
            (ang_data[data_count] > ang_max_)) {
          is_sample_ok = false;
        }
      }

      // update data count of the whole laser scan
      if (is_sample_ok) {
        data_count++;
      }
    }
  }
}

void RPLIDARS2::printPkgDataInfo() const {
  std::cout << std::endl
            << "Start pkg: " << pkg_zero_
            << " #samples: " << sample_count_
            << " Raw start angle: " << raw_ang_data_[0]
            << " Raw end angle: " << raw_ang_data_[sample_count_-1];
  std::cout << "  Start angle: " << ang_data[0] * 180.0f / M_PIf32
            << "  End angle: " << ang_data[sample_count_-1] * 180.0f / M_PIf32
            << std::endl;
}

void RPLIDARS2::printLaserData() const {
  if (data_count > 0) {
    std::cout << std::endl
              << "Laser scan (# points: " << data_count << "):" << std::endl
              << "  Dist (m): ";
    for(size_t i = 0; i < data_count; i++) {
      std::cout << dist_data[i] << " ";
    }
    std::cout << std::endl << "  Angle (deg): ";
    for(size_t i = 0; i < data_count; i++) {
      std::cout << (ang_data[i] * 180.0f / M_PIf32) << " ";
    }
    std::cout << std::endl << std::endl;
  }
}

} // namespace sdpo_driver_laser_2d
