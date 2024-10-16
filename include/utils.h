/**
 * \file utils.h
 * \author Zhihao Zhan (zhanzhihao_dt@163.com)
 * \brief utils
 * \version 0.1
 * \date 2024-10-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef UTILS_H
#define UTILS_H

#include <algorithm>
#include <cmath>
#include <complex>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <json/json.h>
#include <limits>
#include <list>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "alignment.h"

typedef pcl::PointXYZINormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// Check if the given floating point number is a not-a-number (NaN) value.
bool IsNaN(const float x);
bool IsNaN(const double x);

// Check if the given floating point number is a infinity.
bool IsInf(const float x);
bool IsInf(const double x);

// Convert angle in degree to radians.
float DegToRad(const float deg);
double DegToRad(const double deg);

// Convert angle in radians to degree.
float RadToDeg(const float rad);
double RadToDeg(const double rad);

// Determine median value in vector. Returns NaN for empty vectors.
template <typename T> double Median(const std::vector<T> &elems);

// Determine mean value in a vector.
template <typename T> double Mean(const std::vector<T> &elems);

// Determine sample variance in a vector.
template <typename T> double Variance(const std::vector<T> &elems);

// Determine sample standard deviation in a vector.
template <typename T> double StdDev(const std::vector<T> &elems);

/**
 * \brief Get Pose using Isometry3d
 *
 * \param qw
 * \param qx
 * \param qy
 * \param qz
 * \param position
 * \return Eigen::Isometry3d
 */
Eigen::Isometry3d GetPose(double qw, double qx, double qy, double qz,
                          const Eigen::Vector3d &position);

struct Frame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::string fp;                 // file name
  Eigen::Isometry3d T_enu_camera; // extrinsics

  // FOV
  Eigen::Vector3d min_pt;
  Eigen::Vector3d max_pt;
};

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Frame)

#endif // UTILS_H
