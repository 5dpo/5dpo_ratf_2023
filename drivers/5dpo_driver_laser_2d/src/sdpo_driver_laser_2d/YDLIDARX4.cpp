#include "sdpo_driver_laser_2d/YDLIDARX4.h"

#include "sdpo_driver_laser_2d/utils.h"

namespace sdpo_driver_laser_2d {

YDLIDARX4::YDLIDARX4() {
  dist_data.resize(kLaserScanMaxNumSamplesYDLIDARX4);
  ang_data.resize(kLaserScanMaxNumSamplesYDLIDARX4);
}

YDLIDARX4::~YDLIDARX4() {
  if (isSerialOpen()) {
    stop();
  }
  closeSerial();
}

void YDLIDARX4::start() {
  char start_scan_cmd[2];
  start_scan_cmd[0] = (char) 0xA5;
  start_scan_cmd[1] = (char) 0x60;

  serial_async_->write(start_scan_cmd, 2);
}

void YDLIDARX4::stop() {
  char stop_scan_cmd[2];
  stop_scan_cmd[0] = (char) 0xA5;
  stop_scan_cmd[1] = (char) 0x65;

  serial_async_->write(stop_scan_cmd, 2);
}

void YDLIDARX4::restart() {
  char restart_scan_cmd[2];
  restart_scan_cmd[0] = (char) 0xA5;
  restart_scan_cmd[1] = (char) 0x80;

  serial_async_->write(restart_scan_cmd, 2);
}

void YDLIDARX4::processSerialData(unsigned char& ch) {
  // Determine the type of data of the current byte (state transitions)
  switch (state_) {
  case YDLIDARX4State::kPH1:
    if (ch == 0x55) {
      state_ = YDLIDARX4State::kPH2;
    } else {
      state_ = YDLIDARX4State::kIddle;
    }
    break;
  case YDLIDARX4State::kPH2:
    state_ = YDLIDARX4State::kCT;
    break;
  case YDLIDARX4State::kCT:
    state_ = YDLIDARX4State::kLSN;
    break;
  case YDLIDARX4State::kLSN:
    state_ = YDLIDARX4State::kFSA1;
    break;
  case YDLIDARX4State::kFSA1:
    state_ = YDLIDARX4State::kFSA2;
    break;
  case YDLIDARX4State::kFSA2:
    state_ = YDLIDARX4State::kLSA1;
    break;
  case YDLIDARX4State::kLSA1:
    state_ = YDLIDARX4State::kLSA2;
    break;
  case YDLIDARX4State::kLSA2:
    state_ = YDLIDARX4State::kCS1;
    break;
  case YDLIDARX4State::kCS1:
    state_ = YDLIDARX4State::kCS2;
    break;
  case YDLIDARX4State::kCS2:
    state_ = YDLIDARX4State::kSi1;
    sample_count_ = 0;
    break;
  case YDLIDARX4State::kSi1:
    state_ = YDLIDARX4State::kSi2;
    break;
  case YDLIDARX4State::kSi2:
    if (sample_count_ >= pkg_num_samples_) {
      if (ch == 0xAA) {
        state_ = YDLIDARX4State::kPH1;
        processLaserData();
        //printPkgDataInfo();
        byte_count_ = 0;
      } else {
        state_ = YDLIDARX4State::kIddle;
      }
    } else {
      state_ = YDLIDARX4State::kSi1;
    }
    break;
  case YDLIDARX4State::kIddle:
    if (ch == 0xAA) {
      state_ = YDLIDARX4State::kPH1;
      byte_count_ = 0;
    }
    break;
  }

  // Process the current data byte (state actions)
  switch (state_) {
    case YDLIDARX4State::kPH1:
    case YDLIDARX4State::kPH2:
      byte_count_++;
      break;
    case YDLIDARX4State::kCT:
      byte_count_++;
      if ((ch & 0x01) == 0x01) {
        pkg_zero_ = true;
        if (pubLaserData) {
          pubLaserData();
        }
        data_count = 0;
      } else {
        pkg_zero_ = false;
      }
      break;
    case YDLIDARX4State::kLSN:
      byte_count_++;
      pkg_num_samples_ = (ch & 0x00FF);
      break;
    case YDLIDARX4State::kFSA1:
      byte_count_++;
      raw_start_ang_ = (ch & 0x00FF);
      break;
    case YDLIDARX4State::kFSA2:
      byte_count_++;
      raw_start_ang_ = ((raw_start_ang_ | (ch << 8)) >> 1);
      start_ang_ = rawStartEndAng2Double(raw_start_ang_);
      break;
    case YDLIDARX4State::kLSA1:
      byte_count_++;
      raw_end_ang_ = ch & 0x00FF;
      break;
    case YDLIDARX4State::kLSA2:
      byte_count_++;
      raw_end_ang_ = ((raw_end_ang_ | (ch << 8)) >> 1);
      end_ang_ = rawStartEndAng2Double(raw_end_ang_);
      break;
    case YDLIDARX4State::kCS1:
      byte_count_++;
      pkg_check_code_ = ch & 0x00FF;
      break;
    case YDLIDARX4State::kCS2:
      byte_count_++;
      pkg_check_code_ = (pkg_check_code_ | (ch << 8));
      break;
    case YDLIDARX4State::kSi1:
      byte_count_++;
      raw_dist_data_[sample_count_] = ch & 0x00FF;
      break;
    case YDLIDARX4State::kSi2:
      byte_count_++;
      raw_dist_data_[sample_count_] =
          (raw_dist_data_[sample_count_] | (ch << 8));
      sample_count_++;
      break;
    case YDLIDARX4State::kIddle:
      sample_count_ = 0;
      break;
  }
}

void YDLIDARX4::processLaserData() {
  if ((!pkg_zero_) && (sample_count_ > 0)) {
    bool is_sample_ok;
    float angle_correction, delta_ang;

    for(size_t i = 0; i < sample_count_; i++) {
      // distance data (mm)
      dist_data[data_count] = rawDist2Double(raw_dist_data_[i]);

      // angle data
      // - diff angle (end - start)
      delta_ang = end_ang_ - start_ang_;
      if (start_ang_ > end_ang_) {
        delta_ang += 360.0;
      }
      // - angle correction (requires dist data in mm)
      if (raw_dist_data_[i] == 0) {
        angle_correction = 0;
      } else {
        angle_correction =
            atanf(21.8f * (155.3f - dist_data[data_count]) /
                (155.3f * dist_data[data_count])) * 360.0f / M_PIf32;
      }
      // - compute angle
      ang_data[data_count] = normAngRad(
          -(start_ang_ +
          delta_ang * static_cast<float>(i) /
              (static_cast<float>(sample_count_) - 1.0f) +
          angle_correction) * M_PIf32 / 180.0f);

      // distance data (m)
      dist_data[data_count] /= 1000.0f;

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

void YDLIDARX4::printPkgDataInfo() const {
  std::cout << std::endl
            << "Start pkg: " << pkg_zero_
            << " #samples: " << pkg_num_samples_
            << " Raw start angle: " << raw_start_ang_
            << " Raw end angle: " << raw_end_ang_
            << " Checksum: " << pkg_check_code_ << std::endl;
  std::cout << "  Start angle: " << start_ang_
            << "  End angle: " << end_ang_ << std::endl << std::endl;
}

void YDLIDARX4::printLaserData() const {
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
