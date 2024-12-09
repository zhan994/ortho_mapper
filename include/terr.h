/**
 * \file terr.h
 * \author Zhihao Zhan (zhanzhihao_dt@163.com)
 * \brief Terrain from sparse points
 * \version 0.1
 * \date 2024-10-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef TERR_H
#define TERR_H

#include "utils.h"

class Terr {
public:
  Terr();
  ~Terr();

  /**
   * \brief 输入数据
   *
   * \param input 输入点云
   * \param grid_size 网格大小
   * \param perc 高度选取百分比
   * \param kn kdtree近邻点数
   */
  void SetData(const PointCloud::Ptr &input, double grid_size, double perc,
               int kn = 3);

  /**
   * \brief 获取高度
   *
   * \param x enu位置
   * \param y
   * \return double 对应位置的高度
   */
  double GetHeight(double x, double y);

private:
  /**
   * \brief 通过kdtree来插补控制点
   *
   * \param k 近邻点个数
   */
  void KdInterplation(int k);

  /**
   * \brief 体素滤波
   *
   * \param input_cloud
   * \param cloud_filtered
   * \param voxel_size
   */
  void VoxelDownSample(const PointCloud::Ptr &input_cloud,
                       PointCloud::Ptr &cloud_filtered, float voxel_size);

  /**
   * \brief 离群点
   *
   * \param input_cloud
   * \param cloud_filtered
   * \param mean_k
   * \param std_thresh
   */
  void StatisticalRemoveOutlier(const PointCloud::Ptr &input_cloud,
                                PointCloud::Ptr &cloud_filtered, int mean_k,
                                float std_thresh);

  double grid_size_, perc_, kn_;
  PointT min_pt_, max_pt_;
  std::vector<std::vector<PointT>> ct_pts_; // 控制点
  PointCloud::Ptr ct_pts_pcl_;
  int ct_x_num_, ct_y_num_; // xy控制点数
};

#endif // TERR_H