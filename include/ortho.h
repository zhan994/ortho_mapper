/**
 * \file ortho.h
 * \author Zhihao Zhan (zhanzhihao_dt@163.com)
 * \brief orthoimage
 * \version 0.1
 * \date 2024-10-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef ORTHO_H
#define ORTHO_H

#include "gps.h"
#include "terr.h"
#include "utils.h"

#include <tiffio.h>

class OrthoImage {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit OrthoImage(const std::string &cfg);

  /**
   * \brief start mapping
   *
   */
  void Work();

  /**
   * \brief 获取Tiff
   *
   * \param lt_merct_x 左上角mercator xy
   * \param lt_merct_y
   * \return std::string
   */
  std::string GetTiff(double &lt_merct_x, double &lt_merct_y);

private:
  /**
   * \brief load cfg file by jsoncpp
   *
   * \param cfg
   * \return true
   * \return false
   */
  bool LoadCfg(const std::string &cfg);

  /**
   * \brief Get all data in csv
   *
   */
  void GetData();

  /**
   * \brief init tiff
   *
   */
  void InitTiff();

  /**
   * \brief update frame
   *
   * \param frame
   */
  void SetCameraFOV(Frame &frame);

  /**
   * \brief uv to enu
   *
   * \param u
   * \param v
   * \param z
   * \param T_enu_camera
   * \param out
   */
  void UV2ENU(int u, int v, double z, const Eigen::Isometry3d &T_enu_camera,
              Eigen::Vector3d &out);

  /**
   * \brief enu to uv
   *
   * \param T_enu_camera
   * \param point_in_enu
   * \param u
   * \param v
   */
  bool ENU2UV(const Eigen::Isometry3d &T_enu_camera,
              const Eigen::Vector3d &point_in_enu, int &u, int &v);

  /**
   * \brief bound corner pts
   *
   * \param p0
   * \param p1
   * \param p2
   * \param p3
   * \param min_pt
   * \param max_pt
   */
  void BoundCorners(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                    const Eigen::Vector3d &p2, const Eigen::Vector3d &p3,
                    Eigen::Vector3d &min_pt, Eigen::Vector3d &max_pt);

  /**
   * \brief inner-product as score
   *
   * \param point_in_enu
   * \param camera_in_enu
   * \return float
   */
  float ComputeScore(const Eigen::Vector3d &point_in_enu,
                     const Eigen::Vector3d &camera_in_enu);

  /**
   * \brief 合并轨迹在TIFF上
   *
   * \param merged 合并后的数据
   */
  void AddTraj(cv::Mat &merged);

  /**
   * \brief 颜色矫正
   *
   * \param gamma 矫正系数
   */
  void CorrectTiff(double gamma);

  Eigen::Matrix3d camera_K_;
  int img_w_, img_h_;

  std::string txt_path_, img_path_;
  Eigen::Vector3d ori_ell_, ori_ecef_, ori_enu_, ori_merct_;
  std::vector<Frame> frames_;
  PointCloud::Ptr map_pcl_enu_;

  GPSTransform gps_tform_;

  Eigen::Vector3d enu_min_, enu_max_;
  Eigen::Vector3d merct_min_, merct_max_;
  double gsd_;
  int result_width_, result_height_;
  cv::Mat result_img_, score_layer_, traj_img_;

  std::shared_ptr<Terr> terr_ptr_;
  double grid_size_;
  double percent_;
  int ct_kd_interp_;
};

#endif // ORTHO_H