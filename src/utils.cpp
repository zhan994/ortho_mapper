#include "utils.h"

bool IsNaN(const float x) { return x != x; }
bool IsNaN(const double x) { return x != x; }

bool IsInf(const float x) { return !IsNaN(x) && IsNaN(x - x); }
bool IsInf(const double x) { return !IsNaN(x) && IsNaN(x - x); }

float DegToRad(const float deg) {
  return deg * 0.0174532925199432954743716805978692718781530857086181640625f;
}

double DegToRad(const double deg) {
  return deg * 0.0174532925199432954743716805978692718781530857086181640625;
}

// Convert angle in radians to degree.
float RadToDeg(const float rad) {
  return rad * 57.29577951308232286464772187173366546630859375f;
}

double RadToDeg(const double rad) {
  return rad * 57.29577951308232286464772187173366546630859375;
}

template <typename T> double Median(const std::vector<T> &elems) {
  if (elems.empty())
    return 0.;

  const size_t mid_idx = elems.size() / 2;

  std::vector<T> ordered_elems = elems;
  std::nth_element(ordered_elems.begin(), ordered_elems.begin() + mid_idx,
                   ordered_elems.end());

  if (elems.size() % 2 == 0) {
    const T mid_element1 = ordered_elems[mid_idx];
    const T mid_element2 = *std::max_element(ordered_elems.begin(),
                                             ordered_elems.begin() + mid_idx);
    return (mid_element1 + mid_element2) / 2.0;
  } else {
    return ordered_elems[mid_idx];
  }
}

template <typename T> double Mean(const std::vector<T> &elems) {
  if (elems.empty())
    return 0.;

  double sum = 0;
  for (const auto el : elems) {
    sum += static_cast<double>(el);
  }
  return sum / elems.size();
}

template <typename T> double Variance(const std::vector<T> &elems) {
  const double mean = Mean(elems);
  double var = 0;
  for (const auto el : elems) {
    const double diff = el - mean;
    var += diff * diff;
  }
  return var / (elems.size() - 1);
}

template <typename T> double StdDev(const std::vector<T> &elems) {
  return std::sqrt(Variance(elems));
}

Eigen::Isometry3d GetPose(double qw, double qx, double qy, double qz,
                          const Eigen::Vector3d &position) {
  Eigen::Isometry3d T_enu_camera(Eigen::Quaterniond(qw, qx, qy, qz));
  T_enu_camera.translation() = position;
  return T_enu_camera;
}