#pragma once

#include <array>

#include <eigen3/Eigen/Eigen>

#include "sdpo_ratf_ros_localization/SdpoRatfBeacons.h"

namespace sdpo_ratf_ros_localization {

const std::string kSdpoRatfEKFModeOdomWhOnlyStr = "OdomWhOnly";
const std::string kSdpoRatfEKFModeSensOnlyStr = "SensOnly";
const std::string kSdpoRatfEKFModeFusionStr = "Fusion";

class SdpoRatfEKF {
 public:
  enum class SdpoRatfEKFMode {
    kSdpoRatfEKFOdomWhOnly,
    kSdpoRatfEKFSensOnly,
    kSdpoRatfEKFFusion
  };

 public:
  SdpoRatfEKFMode mode;

  Eigen::Vector3d XR, XR_last;
  Eigen::Matrix3d covQ, covP, grad_f_X, grad_f_Q;
  Eigen::Matrix2d covR;
  Eigen::Matrix<double, 2, 3> grad_h_X;

  bool use_last;

 public:
  SdpoRatfEKF();
  ~SdpoRatfEKF() = default;

  void initCovP(const double& p00_ini, const double& p11_ini,
                const double& p22_ini);
  void initCovQ(const double& q00, const double& q11, const double& q22);
  void initCovR(const double& r00, const double& r11);
  void initPose(const double& x_ini, const double& y_ini, const double& th_ini);

  Eigen::Vector3d getRobotPose();
  Eigen::Matrix3d getCovP();
  
  void setUseLastParam(bool useLast);

  void setMode(const SdpoRatfEKFMode& mode_enum);
  void setMode(const std::string& mode_str);
  const SdpoRatfEKFMode getMode();
  const std::string getModeStr();
  
  void predict(const double& dd, const double& ddn, const double& dth);
  void update(const SdpoRatfBeacons& beacon_obs,
      const std::array<double,2>& beacon_gt);

 private:
  void updateOdom(const double& dd, const double& ddn, const double& dth);
};

} // namespace sdpo_ratf_ros_localization