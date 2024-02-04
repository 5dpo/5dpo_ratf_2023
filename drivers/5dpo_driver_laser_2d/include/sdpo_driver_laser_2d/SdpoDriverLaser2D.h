#pragma once

#include <iostream>
#include <functional>
#include <vector>

#include <sdpo_ros_serial_port/AsyncSerial.h>

namespace sdpo_driver_laser_2d {

class SdpoDriverLaser2D {
 public:
  std::vector<float> dist_data;
  std::vector<float> ang_data;
  size_t data_count = 0;

  const boost::asio::serial_port_base::character_size
      kSerialDataBits = boost::asio::serial_port_base::character_size(8);
  const boost::asio::serial_port_base::stop_bits::type
      kSerialStopBits = boost::asio::serial_port_base::stop_bits::one;
  const boost::asio::serial_port_base::parity::type
      kSerialParity = boost::asio::serial_port_base::parity::none;
  const boost::asio::serial_port_base::flow_control::type
      kSerialFlowCtrl = boost::asio::serial_port_base::flow_control::none;

 protected:
  bool dist_range_check_ = false;
  float dist_min_;
  float dist_max_;
  bool ang_range_check_ = false;
  float ang_min_;
  float ang_max_;

  std::string serial_port_name_;
  int baud_rate_;
  CallbackAsyncSerial *serial_async_;

  std::function<void()> pubLaserData;

 public:
  SdpoDriverLaser2D();
  virtual ~SdpoDriverLaser2D() = default;

  bool openSerial(const bool dbg = false);
  void closeSerial(const bool dbg = false);
  bool isSerialOpen();

  void setSerialPortParam(
      const std::string& serial_port_name, const int& baud_rate);
  void setDistRangeCheck(const float& dist_min, const float& dist_max);
  void setAngRangeCheck(const float& ang_min, const float& ang_max);
  void setPubLaserData(const std::function<void()>& pubLaserDataFunction);

  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void restart() = 0;

 protected:
  void rcvSerialData(const char *data, unsigned int len);
  virtual void processSerialData(unsigned char& ch) = 0;
  virtual void processLaserData() = 0;

  virtual void printPkgDataInfo() const = 0;
  virtual void printLaserData() const = 0;
};

} // namespace sdpo_driver_laser_2d
