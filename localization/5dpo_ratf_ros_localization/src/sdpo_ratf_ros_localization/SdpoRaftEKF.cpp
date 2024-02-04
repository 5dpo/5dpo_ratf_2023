#include "sdpo_ratf_ros_localization/SdpoRatfEKF.h"

#include "sdpo_ratf_ros_localization/utils.h"

namespace sdpo_ratf_ros_localization {

SdpoRatfEKF::SdpoRatfEKF() {
  XR = Eigen::Vector3d::Zero();
  XR_last = Eigen::Vector3d::Zero();

  covQ = Eigen::Matrix3d::Zero();
  covR = Eigen::Matrix2d::Zero();
  covP = Eigen::Matrix3d::Zero();

  grad_f_X = Eigen::Matrix3d::Identity();

  grad_f_Q = Eigen::Matrix3d::Zero();
  grad_f_Q(2,2) = 1;

  grad_h_X = Eigen::Matrix<double, 2, 3>::Zero();
  grad_h_X(1,2) = -1;

  use_last = false;
  mode = SdpoRatfEKFMode::kSdpoRatfEKFFusion;
}

void SdpoRatfEKF::initCovP(const double& p00_ini, const double& p11_ini,
                           const double& p22_ini) {
  covP = Eigen::Matrix3d::Zero();
  covP(0,0) = p00_ini;
  covP(1,1) = p11_ini;
  covP(2,2) = p22_ini;
}

void SdpoRatfEKF::initCovQ(const double& q00, const double& q11,
                           const double& q22) {
  covQ(0,0) = q00; // Distance measure covariance 
  covQ(1,1) = q11; 
  covQ(2,2) = q22; // Orientation measure covariance
}

void SdpoRatfEKF::initCovR(const double& r00, const double& r11) {
  covR(0,0) = r00; // Distance observation noise covariance
  covR(1,1) = r11; // Orientation observation noise covariance
}

void SdpoRatfEKF::initPose(const double& x_ini, const double& y_ini,
                           const double& th_ini) {
  XR(0) = x_ini;
  XR(1) = y_ini;
  XR(2) = th_ini;
}

Eigen::Vector3d SdpoRatfEKF::getRobotPose() {
  return XR;
}

Eigen::Matrix3d SdpoRatfEKF::getCovP() {
  return covP;
}

void SdpoRatfEKF::setUseLastParam(bool useLast) {
  use_last = useLast;
}

void SdpoRatfEKF::setMode(const SdpoRatfEKFMode& mode_enum) {
  mode = mode_enum;
}

void SdpoRatfEKF::setMode(const std::string& mode_str) {
  if (mode_str == kSdpoRatfEKFModeOdomWhOnlyStr) {
    mode = SdpoRatfEKFMode::kSdpoRatfEKFOdomWhOnly;
  } else if (mode_str == kSdpoRatfEKFModeSensOnlyStr) {
    mode = SdpoRatfEKFMode::kSdpoRatfEKFSensOnly;
  } else {
    mode = SdpoRatfEKFMode::kSdpoRatfEKFFusion;
  }
}

const SdpoRatfEKF::SdpoRatfEKFMode SdpoRatfEKF::getMode() {
  return mode;
}

const std::string SdpoRatfEKF::getModeStr() {
  if (mode == SdpoRatfEKFMode::kSdpoRatfEKFOdomWhOnly) {
    return kSdpoRatfEKFModeOdomWhOnlyStr;
  } else if (mode == SdpoRatfEKFMode::kSdpoRatfEKFSensOnly) {
    return kSdpoRatfEKFModeSensOnlyStr;
  } else if (mode == SdpoRatfEKFMode::kSdpoRatfEKFFusion) {
    return kSdpoRatfEKFModeFusionStr;
  } else {
    return "Undefined";
  }
}

void SdpoRatfEKF::predict(const double& dd, const double& ddn,
    const double& dth) {
  if ((mode == SdpoRatfEKFMode::kSdpoRatfEKFFusion) ||
      (mode == SdpoRatfEKFMode::kSdpoRatfEKFOdomWhOnly)) {
    updateOdom(dd, ddn, dth);

    grad_f_X(0,2) = -dd * sin(XR(2)) - ddn * cos(XR(2));
    grad_f_X(1,2) =  dd * cos(XR(2)) - ddn * sin(XR(2));

    grad_f_Q(0,0) =  cos(XR(2));
    grad_f_Q(0,1) = -sin(XR(2));
    grad_f_Q(1,0) =  sin(XR(2));
    grad_f_Q(1,1) =  cos(XR(2));

    covP = grad_f_X * covP * grad_f_X.transpose() +
          grad_f_Q * covQ * grad_f_Q.transpose();
  }
}

void SdpoRatfEKF::update(const SdpoRatfBeacons& beacon_obs,
    const std::array<double,2>& beacon_gt) {
  if ((mode == SdpoRatfEKFMode::kSdpoRatfEKFFusion) ||
      (mode == SdpoRatfEKFMode::kSdpoRatfEKFSensOnly)) {
    double dBeacon, ang_expected;
    Eigen::Vector2d Z_E, Maux;
    Eigen::Matrix<double, 3, 2> Kf;

    // Calculate the distance from the robot to the beacon
    Maux = Eigen::Vector2d::Zero();
    Maux(0) = beacon_gt[0] - XR(0);
    Maux(1) = beacon_gt[1] - XR(1);
    dBeacon = sqrt(std::pow(Maux(0), 2) + std::pow(Maux(1), 2));

    // Calculate the Jacobian of h() with respect to the state
    if(use_last) {
      grad_h_X(0,0) = -(beacon_gt[0] - XR_last(0))/dBeacon;
      grad_h_X(1,0) =  (beacon_gt[1] - XR_last(1))/std::pow(dBeacon,2);
      grad_h_X(0,1) = -(beacon_gt[1] - XR_last(1))/dBeacon;
      grad_h_X(1,1) = -(beacon_gt[0] - XR_last(0))/std::pow(dBeacon,2);
    }
    else {
      grad_h_X(0,0) = -(beacon_gt[0] - XR(0))/dBeacon;
      grad_h_X(1,0) =  (beacon_gt[1] - XR(1))/std::pow(dBeacon,2);
      grad_h_X(0,1) = -(beacon_gt[1] - XR(1))/dBeacon;
      grad_h_X(1,1) = -(beacon_gt[0] - XR(0))/std::pow(dBeacon,2);
    }
    
    // Kalman Filter Gain
    Kf = covP * grad_h_X.transpose() *
        (grad_h_X * covP * grad_h_X.transpose() + covR).inverse();

    // Update the state covariance
    covP = (Eigen::Matrix3d::Identity() - Kf * grad_h_X) * covP;

    // Calculate the error between measured and estimated distances
    Z_E = Eigen::Vector2d::Zero();
    Z_E(0) = beacon_obs.dist - dBeacon;
    Z_E(1) = normAngRad(beacon_obs.ang -
        normAngRad(atan2(beacon_gt[1] - XR(1), beacon_gt[0] - XR(0)) - XR(2)));

    // Correct state estimation
    XR = XR + Kf * Z_E;
  }
}

void SdpoRatfEKF::updateOdom(const double& dd, const double& ddn,
    const double& dth) {
  XR(0) += dd * cos(XR(2)) - ddn * sin(XR(2));
  XR(1) += dd * sin(XR(2)) + ddn * cos(XR(2));
  XR(2) += dth;
  XR_last = XR;
}

} // namespace sdpo_ratf_ros_localization