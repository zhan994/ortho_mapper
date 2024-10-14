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
#include <json/json.h>
#include <limits>
#include <list>
#include <stdexcept>

#include "alignment.h"

// Check if the given floating point number is a not-a-number (NaN) value.
inline bool IsNaN(const float x);
inline bool IsNaN(const double x);

// Check if the given floating point number is a infinity.
inline bool IsInf(const float x);
inline bool IsInf(const double x);

// Convert angle in degree to radians.
inline float DegToRad(const float deg);
inline double DegToRad(const double deg);

// Convert angle in radians to degree.
inline float RadToDeg(const float rad);
inline double RadToDeg(const double rad);

// Determine median value in vector. Returns NaN for empty vectors.
template <typename T> double Median(const std::vector<T> &elems);

// Determine mean value in a vector.
template <typename T> double Mean(const std::vector<T> &elems);

// Determine sample variance in a vector.
template <typename T> double Variance(const std::vector<T> &elems);

// Determine sample standard deviation in a vector.
template <typename T> double StdDev(const std::vector<T> &elems);

#endif // UTILS_H
