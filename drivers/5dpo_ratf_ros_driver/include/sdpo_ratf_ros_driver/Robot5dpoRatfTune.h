#pragma once

#include <iostream>
#include <functional>
#include <mutex>

#include <sdpo_ros_serial_port/AsyncSerial.h>

#include "sdpo_ratf_ros_driver/SerialChannelsConfig.h"
#include "sdpo_ratf_ros_driver/Robot5dpoRatf.h"

namespace sdpo_ratf_ros_driver {

class Robot5dpoRatfTune {
 public:
  Motor mot[4];
  std::mutex mtx_;

  bool switch_state = false;
  bool solenoid_state = false;

  std::function<void()> run;

 private:
  std::string serial_port_name_;
  SerialChannelsConfig *serial_cfg_;
  CallbackAsyncSerial *serial_async_;

 public:
  Robot5dpoRatfTune();
  Robot5dpoRatfTune(std::string serial_port_name);
  ~Robot5dpoRatfTune();

  void init();

  bool openSerial(const bool dbg = false);
  void closeSerial(const bool dbg = false);
  bool isSerialOpen();

  void setSerialPortName(const std::string& serial_port_name);

  void reset();

  void stopMotors();

 private:
  void rcvSerialData(const char *data, unsigned int len);
  void sendSerialData();
};

} // namespace sdpo_ratf_ros_driver
