#include "sdpo_ratf_ros_driver/Robot5dpoRatfTune.h"

#include <exception>
#include <utility>
#include <vector>

namespace sdpo_ratf_ros_driver {

const unsigned int kSerialBaudRate = 115200;
const auto kSerialDataBits = boost::asio::serial_port_base::character_size(8);
const auto kSerialStopBits = boost::asio::serial_port_base::stop_bits::one;
const auto kSerialParity = boost::asio::serial_port_base::parity::none;
const auto kSerialFlowCtrl = boost::asio::serial_port_base::flow_control::none;

Robot5dpoRatfTune::Robot5dpoRatfTune() : serial_async_(nullptr) {
  serial_cfg_ = InitCommunications();
}

Robot5dpoRatfTune::Robot5dpoRatfTune(std::string serial_port_name)
    : serial_async_(nullptr),
      serial_port_name_(std::move(serial_port_name)) {
  serial_cfg_ = InitCommunications();
}

Robot5dpoRatfTune::~Robot5dpoRatfTune() {
  closeSerial();
}

void Robot5dpoRatfTune::init() {
  reset();
  if (isSerialOpen()) {
    sendSerialData();
  }
}

bool Robot5dpoRatfTune::openSerial(const bool dbg) {
  try {
    serial_async_ = new CallbackAsyncSerial(
        serial_port_name_, kSerialBaudRate,
        boost::asio::serial_port_base::parity(kSerialParity),
        kSerialDataBits,
        boost::asio::serial_port_base::flow_control(kSerialFlowCtrl),
        boost::asio::serial_port_base::stop_bits(kSerialStopBits));
    serial_async_->setCallback(
        std::bind(&Robot5dpoRatfTune::rcvSerialData, this,
                  std::placeholders::_1, std::placeholders::_2));
    return true;
  } catch (boost::system::system_error& e) {
    serial_async_ = nullptr;
    if (dbg) {
      std::cerr << "[Robot5dpoRatfTune.cpp] Robot5dpoRatfTune::openSerial: "
                   "Error when opening the serial device "
                << serial_port_name_ << std::endl;
    }
    return false;
  }
}

void Robot5dpoRatfTune::closeSerial(const bool dbg) {
  if (serial_async_) {
    try {
      serial_async_->close();
    } catch(...) {
      if (dbg) {
        std::cerr << "[Robot5dpoRatfTune.cpp] Robot5dpoRatfTune::closeSerial: "
                     "Error when closing the serial device "
                  << serial_port_name_ << std::endl;
      }
    }
    delete serial_async_;
  }
}

bool Robot5dpoRatfTune::isSerialOpen() {
  return serial_async_ ?
         !(serial_async_->errorStatus() || (!serial_async_->isOpen())) : false;
}

void Robot5dpoRatfTune::setSerialPortName(const std::string& serial_port_name) {
  serial_port_name_ = serial_port_name;
}

void Robot5dpoRatfTune::reset() {
  for(auto& m : mot) {
    m.reset();
  }
}

void Robot5dpoRatfTune::stopMotors() {
  for(auto& m : mot) {
    m.w_r = 0;
  }
}

void Robot5dpoRatfTune::rcvSerialData(const char *data, unsigned int len) {
  std::vector<char> vec(data, data + len);
  char channel;

  for(auto& c : vec) {
    if (!((c < 32) || (c >= 0x7f))) { // ignore non-ascii char
      channel = ProcessChannelsSerialData(c);

      if (channel > 0) {
        switch (channel) {
        case 'g':
          sendSerialData();
          if (run) {
            run();
          }
          mtx_.lock();
          mot[0].setEncTicks(serial_cfg_->channel_g);
          mtx_.unlock();
          break;

        case 'h':
          mtx_.lock();
          mot[1].setEncTicks(serial_cfg_->channel_h);
          mtx_.unlock();
          break;

        case 'i':
          mtx_.lock();
          mot[2].setEncTicks(serial_cfg_->channel_i);
          mtx_.unlock();
          break;

        case 'j':
          mtx_.lock();
          mot[3].setEncTicks(serial_cfg_->channel_j);
          mtx_.unlock();
          break;

        case 'k':
          mtx_.lock();
          for(auto& m : mot) {
            m.setSampleTime(serial_cfg_->channel_k);
          }
          mtx_.unlock();
          break;

        case 'r':
          mtx_.lock();
          reset();
          mtx_.unlock();
          break;

        case 's':
          mtx_.lock();
          switch_state = (serial_cfg_->channel_s == 0);
          mtx_.unlock();

        default:
          break;
        }
      }
    }
  }
}

void Robot5dpoRatfTune::sendSerialData() {
  mtx_.lock();
  for (uint8_t i = 0; i < 4; i++) {
    serial_cfg_->channel_K = mot[i].pwm & 0xFFFF;
    serial_cfg_->channel_K = serial_cfg_->channel_K |
        ((i & 0x03) << 24);
    serial_async_->writeString(SendChannel('K'));
  }
  serial_cfg_->channel_L = solenoid_state? 1 : 0;
  mtx_.unlock();

  serial_async_->writeString(SendChannel('L'));
}

} // namespace sdpo_ratf_ros_driver