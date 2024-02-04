#pragma once

#include <iostream>
#include <cmath>

namespace sdpo_ratf_ros_localization {

inline float dist(float x, float y) {
  return sqrtf(x * x + y * y);
}

inline double dist(double x, double y) {
  return sqrt(x * x + y * y);
}

inline float normAngDeg(float angle) {
  // Source: https://stackoverflow.com/a/11498248
  angle = fmodf(angle + 180.0f, 360.0f);
  if (angle < 0) {
    angle += 360.0f;
  }
  return angle - 180.0f;
}

inline double normAngDeg(double angle) {
  // Source: https://stackoverflow.com/a/11498248
  angle = fmod(angle + 180.0, 360.0);
  if (angle < 0) {
    angle += 360.0;
  }
  return angle - 180.0;
}

inline float normAngRad(float angle) {
  // Source: https://stackoverflow.com/a/11498248
  angle = fmodf(angle + M_PIf32, M_PIf32 * 2.0f);
  if (angle < 0) {
    angle += M_PIf32 * 2.0f;
  }
  return angle - M_PIf32;
}

inline double normAngRad(double angle) {
  // Source: https://stackoverflow.com/a/11498248
  angle = fmod(angle + M_PI, M_PI * 2.0);
  if (angle < 0) {
    angle += M_PI * 2.0;
  }
  return angle - M_PI;
}

inline double deg2rad(double angle) {
  return angle * M_PI / 180.0;
}

inline double rad2deg(double angle) {
  return angle * 180.0 / M_PI;
}

} // namespace sdpo_ratf_ros_localization
