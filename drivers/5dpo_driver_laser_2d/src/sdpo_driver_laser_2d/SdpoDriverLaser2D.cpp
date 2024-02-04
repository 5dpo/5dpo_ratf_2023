#include "sdpo_driver_laser_2d/SdpoDriverLaser2D.h"

namespace sdpo_driver_laser_2d{

SdpoDriverLaser2D::SdpoDriverLaser2D() : serial_async_(nullptr) { }

bool SdpoDriverLaser2D::openSerial(const bool dbg) {
  try {
    serial_async_ = new CallbackAsyncSerial(
        serial_port_name_, 115200,
        boost::asio::serial_port_base::parity(kSerialParity),
        kSerialDataBits,
        boost::asio::serial_port_base::flow_control(kSerialFlowCtrl),
        boost::asio::serial_port_base::stop_bits(kSerialStopBits));
    serial_async_->setCustomBaudRate(baud_rate_);
    serial_async_->setCallback(
        std::bind(&SdpoDriverLaser2D::rcvSerialData, this,
                  std::placeholders::_1, std::placeholders::_2));
    return true;
  } catch (boost::system::system_error& e) {
    serial_async_ = nullptr;
    if (dbg) {
      std::cerr << "[YDLIDARX4.cpp] YDLIDARX4::openSerial: "
                   "Error when opening the serial device "
                << serial_port_name_ << std::endl;
    }
    return false;
  } catch (std::exception& e) {
    serial_async_ = nullptr;
    if (dbg) {
      std::cerr << "[YDLIDARX4.cpp] YDLIDARX4::openSerial: "
                   "Error when setting the custom baud rate of the serial "
                   "device "
                << serial_port_name_ << "(error: "
                << e.what() << ")" << std::endl;
    }
    return false;
  }
}

void SdpoDriverLaser2D::closeSerial(const bool dbg) {
  if (serial_async_) {
    try {
      serial_async_->close();
    } catch(...) {
      if (dbg) {
        std::cerr << "[YDLIDARX4.cpp] YDLIDARX4::closeSerial: "
                     "Error when closing the serial device "
                  << serial_port_name_ << std::endl;
      }
    }
    delete serial_async_;
  }
}

bool SdpoDriverLaser2D::isSerialOpen() {
  return serial_async_ ?
         !(serial_async_->errorStatus() || (!serial_async_->isOpen())) : false;
}

void SdpoDriverLaser2D::setSerialPortParam(
    const std::string& serial_port_name, const int& baud_rate) {
  serial_port_name_ = serial_port_name;
  baud_rate_ = baud_rate;
}

void SdpoDriverLaser2D::setDistRangeCheck(
    const float& dist_min, const float& dist_max) {
  dist_range_check_ = true;
  dist_min_ = dist_min;
  dist_max_ = dist_max;
}

void SdpoDriverLaser2D::setAngRangeCheck(
    const float& ang_min, const float& ang_max) {
  ang_range_check_ = true;
  ang_min_ = ang_min;
  ang_max_ = ang_max;
}

void SdpoDriverLaser2D::setPubLaserData(
    const std::function<void()>& pubLaserDataFunction) {
  pubLaserData = pubLaserDataFunction;
}

void SdpoDriverLaser2D::rcvSerialData(const char *data, unsigned int len) {
  std::vector<unsigned char> vec(data, data + len);

  for(unsigned char& c : vec) {
    //printf("%X ", (unsigned char) c);
    processSerialData(c);
  }
}

} // namespace sdpo_driver_laser_2d
