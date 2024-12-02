/**
 * \file gps.h
 * \author Zhihao Zhan (zhanzhihao_dt@163.com)
 * \brief gps trans (from gps.h in colmap)
 * \version 0.1
 * \date 2024-10-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef GPS_H
#define GPS_H

#include "utils.h"

class GPSTransform {
public:
  enum ELLIPSOID { GRS80, WGS84 };

  explicit GPSTransform(const int ellipsoid = WGS84);

  /**
   * \brief Convert GPS (lat / lon / alt) to ECEF
   * 
   * \param ell 
   * \return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d>
  EllToXYZ(const std::vector<Eigen::Vector3d> &ell) const;

  /**
   * \brief Convert ECEF to GPS (lat / lon / alt)
   * 
   * \param xyz 
   * \return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d>
  XYZToEll(const std::vector<Eigen::Vector3d> &xyz) const;

  // Convert GPS (lat / lon / alt) to ENU coords. with lat0 and lon0
  // defining the origin of the ENU frame
  /**
   * \brief Convert GPS (lat / lon / alt) to ENU coords. 
   * with lat0 and lon0 defining the origin of the ENU frame (ell[0])
   * 
   * \param ell 
   * \param lat0 
   * \param lon0 
   * \return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> EllToENU(const std::vector<Eigen::Vector3d> &ell,
                                        const double lat0,
                                        const double lon0) const;

  /**
   * \brief Convert ECEF to ENU coords. 
   * with lat0 and lon0 defining the origin of the ENU frame (xyz[0])
   * 
   * \param xyz 
   * \param lat0 
   * \param lon0 
   * \return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> XYZToENU(const std::vector<Eigen::Vector3d> &xyz,
                                        const double lat0,
                                        const double lon0) const;

  /**
   * \brief Convert ENU to GPS (lat / lon / alt) coords. 
   * with lat0 and lon0 defining the origin of the ENU frame
   * 
   * \param enu 
   * \param lat0 
   * \param lon0 
   * \param alt0 
   * \return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> ENUToEll(const std::vector<Eigen::Vector3d> &enu,
                                        const double lat0, const double lon0,
                                        const double alt0) const;

  /**
   * \brief Convert ENU to ECEF coords. 
   * with lat0 and lon0 defining the origin of the ENU frame
   * 
   * 
   * \param enu 
   * \param lat0 
   * \param lon0 
   * \param alt0 
   * \return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> ENUToXYZ(const std::vector<Eigen::Vector3d> &enu,
                                        const double lat0, const double lon0,
                                        const double alt0) const;

private:
  // Semimajor axis.
  double a_;
  // Semiminor axis.
  double b_;
  // Flattening.
  double f_;
  // Numerical eccentricity.
  double e2_;
};

#endif // GPS_H