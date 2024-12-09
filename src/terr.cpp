#include "terr.h"
Terr::Terr() : ct_pts_pcl_(new PointCloud) {
  std::cout << "Cons. BSpline Surface." << std::endl;
}

Terr::~Terr() {}

void Terr::SetData(const PointCloud::Ptr &input, double grid_size, double perc,
                   int kn) {
  std::cout << "Init,,," << std::endl;
  grid_size_ = grid_size;
  perc_ = perc;
  kn_ = kn;

  PointCloud::Ptr data(new PointCloud);
  VoxelDownSample(input, data, 1.0);
  StatisticalRemoveOutlier(data, data, 10, 2.0);

  ct_pts_.clear();

  // step: 计算数据范围
  pcl::getMinMax3D(*data, min_pt_, max_pt_);
  std::cout << "min_x: " << min_pt_.x << " "
            << "max_x: " << max_pt_.x << std::endl;
  std::cout << "min_y: " << min_pt_.y << " "
            << "max_y: " << max_pt_.y << std::endl;

  double range_x = max_pt_.x - min_pt_.x;
  double range_y = max_pt_.y - min_pt_.y;
  ct_x_num_ = std::floor(range_x / grid_size_) + 1;
  ct_y_num_ = std::floor(range_y / grid_size_) + 1;
  std::vector<std::vector<PointCloud>> grids(
      ct_x_num_, std::vector<PointCloud>(ct_y_num_));
  ct_pts_ = std::vector<std::vector<PointT>>(ct_x_num_,
                                             std::vector<PointT>(ct_y_num_));

  // step: 点云分组，对每组点云进行高度提取
  int invalid = 0;
  for (const auto &pt : *data) {
    int x_ind = std::floor((pt.x - min_pt_.x) / grid_size_);
    int y_ind = std::floor((pt.y - min_pt_.y) / grid_size_);

    if (x_ind >= 0 && x_ind < ct_x_num_ && y_ind >= 0 && y_ind < ct_y_num_)
      grids[x_ind][y_ind].points.push_back(pt);
  }

  for (int i = 0; i < ct_x_num_; ++i) {
    for (int j = 0; j < ct_y_num_; ++j) {
      auto &pts = grids[i][j].points;
      PointT ct_pt;
      ct_pt.x = (i + 0.5) * grid_size_ + min_pt_.x;
      ct_pt.y = (j + 0.5) * grid_size_ + min_pt_.y;
      if (!pts.empty()) {
        std::sort(pts.begin(), pts.end(),
                  [](PointT a, PointT b) { return a.z < b.z; });
        int sel_ind = pts.size() * perc;
        ct_pt.z = pts[sel_ind].z;
        ct_pts_pcl_->points.push_back(ct_pt);
      } else {
        ct_pt.z = std::numeric_limits<double>::infinity();
        invalid++;
      }

      ct_pts_[i][j] = ct_pt;
    }
  }

  // step: 对空缺的控制点进行插值
  if (invalid > 0)
    KdInterplation(kn);

  pcl::io::savePLYFile("terr.ply", *ct_pts_pcl_);
  std::cout << "Finish Init." << std::endl;
}

void Terr::KdInterplation(int kn) {
  std::cout << "Interpolation..." << std::endl;
  // step: 提取2d点
  PointCloud::Ptr ct_pts_pcl_2d(new PointCloud);
  for (const auto &pt : *ct_pts_pcl_) {
    PointT pt_2d;
    pt_2d.x = pt.x;
    pt_2d.y = pt.y;
    pt_2d.z = 0.;
    ct_pts_pcl_2d->points.push_back(pt_2d);
  }

  // step: 对于失效高度进行插值
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(ct_pts_pcl_2d);
  std::vector<int> indices(kn);
  std::vector<float> sq_dists(kn);
  for (int i = 0; i < ct_x_num_; ++i) {
    for (int j = 0; j < ct_y_num_; ++j) {
      PointT tgt_pt = ct_pts_[i][j];
      if (std::isinf(tgt_pt.z)) {
        tgt_pt.z = 0.0;
        if (kdtree.nearestKSearch(tgt_pt, kn, indices, sq_dists) > 0) {
          double h_sum = 0;

          for (const auto &ind : indices)
            h_sum += ct_pts_pcl_->points.at(ind).z;

          ct_pts_[i][j].z = h_sum / kn;
          ct_pts_pcl_->points.push_back(ct_pts_[i][j]);
        }
      }
    }
  }

  std::cout << "Finish Interpolation." << std::endl;
}

double Terr::GetHeight(double x, double y) {
  int x_ind = std::floor((x - min_pt_.x) / grid_size_);
  int y_ind = std::floor((y - min_pt_.y) / grid_size_);

  if (x_ind >= 0 && x_ind < ct_x_num_ && y_ind >= 0 && y_ind < ct_y_num_) {
    PointT pt = ct_pts_[x_ind][y_ind];
    return pt.z;
  } else {
    // std::cout << "x_ind: " << x_ind << " y_ind: " << y_ind << std::endl;
    // std::cout << "ct_x_num_: " << ct_x_num_ << " ct_y_num_: " << ct_y_num_
    //           << std::endl;
    return std::numeric_limits<double>::infinity();
  }
}

void Terr::VoxelDownSample(const PointCloud::Ptr &input_cloud,
                           PointCloud::Ptr &cloud_filtered, float voxel_size) {
  // Create the filtering object
  pcl::VoxelGrid<PointT> sor;

  sor.setInputCloud(input_cloud);
  sor.setLeafSize(voxel_size, voxel_size, voxel_size); // voxel_size
  // sor.setLeafSize(0.5f, 0.5f, 0.5f);  // voxel_size
  sor.filter(*cloud_filtered);
}

void Terr::StatisticalRemoveOutlier(const PointCloud::Ptr &input_cloud,
                                    PointCloud::Ptr &cloud_filtered, int mean_k,
                                    float std_thresh) {
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(input_cloud);
  sor.setMeanK(mean_k);               // default: 10 越大越严格
  sor.setStddevMulThresh(std_thresh); // default: 2.0 越小越严格
  sor.filter(*cloud_filtered);
}
