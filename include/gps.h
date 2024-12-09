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
   * \param ell GPS(lat / lon / alt)
   * \return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d>
  EllToXYZ(const std::vector<Eigen::Vector3d> &ell) const;

  /**
   * \brief Convert ECEF to GPS (lat / lon / alt)
   *
   * \param xyz ECEF(x / y / z)
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
   * \param ell GPS(lat / lon / alt)
   * \param lat0 ell[0](0)
   * \param lon0 ell[0](1)
   * \return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> EllToENU(const std::vector<Eigen::Vector3d> &ell,
                                        const double lat0,
                                        const double lon0) const;

  /**
   * \brief Convert ECEF to ENU coords.
   * with lat0 and lon0 defining the origin of the ENU frame (xyz[0])
   *
   * \param xyz ECEF(x / y / z)
   * \param lat0 ell[0](0)
   * \param lon0 ell[0](1)
   * \return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> XYZToENU(const std::vector<Eigen::Vector3d> &xyz,
                                        const double lat0,
                                        const double lon0) const;

  /**
   * \brief Convert ENU to GPS (lat / lon / alt) coords.
   * with lat0 and lon0 defining the origin of the ENU frame
   *
   * \param enu ENU(x / y / z)
   * \param lat0 ell[0](0)
   * \param lon0 ell[0](1)
   * \param alt0 ell[0](2)
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
   * \param enu ENU(x / y / z)
   * \param lat0 ell[0](0)
   * \param lon0 ell[0](1)
   * \param alt0 ell[0](2)
   * \return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> ENUToXYZ(const std::vector<Eigen::Vector3d> &enu,
                                        const double lat0, const double lon0,
                                        const double alt0) const;

  /**
   * \brief Convert GPS (lat / lon / alt) to Mercator coords.
   *
   * \param ell GPS(lat / lon)
   * \return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d>
  EllToMercator(const std::vector<Eigen::Vector3d> &ell) const;

  /**
   * \brief Convert ECEF to Mercator coords.
   *
   * \param xyz ECEF(x / y / z)
   * \return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d>
  XYZToMercator(const std::vector<Eigen::Vector3d> &xyz) const;

  /**
   * \brief Convert ENU to Mercator coords.
   *
   * \param enu ENU(x / y / z)
   * \param lat0 ell[0](0)
   * \param lon0 ell[0](1)
   * \param alt0 ell[0](2)
   * \return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d>
  ENUToMercator(const std::vector<Eigen::Vector3d> &enu, const double lat0,
                const double lon0, const double alt0) const;

  /**
   * \brief Convert Mercator to GPS (lat / lon / alt) coords.
   *
   * \param mercator
   * \return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d>
  MercatorToEll(const std::vector<Eigen::Vector3d> &mercator) const;

  /**
   * \brief Convert Mercator to ECEF coords.
   *
   * \param mercator
   * \return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d>
  MercatorToXYZ(const std::vector<Eigen::Vector3d> &mercator) const;

  /**
   * \brief 
   * 
   * \param mercator 
   * \param lat0 
   * \param lon0 
   * \return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d>
  MercatorToENU(const std::vector<Eigen::Vector3d> &mercator, const double lat0,
                const double lon0) const;

  /**
   * \brief Convert ECEF to ENU coords.
   * with lat0 and lon0 defining the origin of the ENU frame (xyz[0])
   *
   * \param rotation_ecef ecef rotation
   * \param lat0 ell[0](0)
   * \param lon0 ell[0](1)
   * \return std::vector<Eigen::Quaterniond>
   */
  std::vector<Eigen::Quaterniond>
  XYZToENURotation(const std::vector<Eigen::Quaterniond> &rotation_ecef,
                   const double lat0, const double lon0) const;

  /**
   * \brief Convert ENU to ECEF coords.
   * with lat0 and lon0 defining the origin of the ENU frame (enu[0])
   *
   * \param rotation_enu enu rotation
   * \param lat0 ell[0](0)
   * \param lon0 ell[0](1)
   * \return std::vector<Eigen::Quaterniond>
   */
  std::vector<Eigen::Quaterniond>
  ENUToXYZRotation(const std::vector<Eigen::Quaterniond> &rotation_enu,
                   const double lat0, const double lon0) const;

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