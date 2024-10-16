#include "ortho.h"

OrthoImage::OrthoImage(const std::string &cfg) : map_pcl_(new PointCloud) {
  std::cout << "Cons. OrthoImage: " << cfg << std::endl;

  if (LoadCfg(cfg)) {
    std::cout << "Load cfg done." << std::endl;
    GetData();
  }

  std::cout << "Cons. OrthoImage done." << std::endl;
}

bool OrthoImage::LoadCfg(const std::string &cfg) {
  Json::Reader reader;
  Json::Value value;
  std::ifstream in(cfg, std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "Open config file failed!" << std::endl;
    return false;
  }

  if (reader.parse(in, value)) {
    txt_path_ = value["txt_path"].asString();
    img_path_ = value["img_path"].asString();
    terr_path_ = value["terr_path"].asString();

    img_w_ = value["img_size"][0].asInt();
    img_h_ = value["img_size"][1].asInt();

    double fx = value["intrinsics"][0].asDouble();
    double fy = value["intrinsics"][1].asDouble();
    double cx = value["intrinsics"][2].asDouble();
    double cy = value["intrinsics"][3].asDouble();
    camera_K_ << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    gsd_ = value["gsd"].asDouble();
    ku_ = value["terr"]["ku"].asInt();
    kv_ = value["terr"]["kv"].asInt();
    grid_size_ = value["terr"]["grid_size"].asDouble();
    percent_ = value["terr"]["percent"].asDouble();
    ct_kd_interp_ = value["terr"]["ct_kd_interp"].asInt();
  } else {
    std::cerr << "Can not parse Json file!" << std::endl;
    return false;
  }
  std::cout << "========> Parse Json file done." << std::endl;
  return true;
}

void OrthoImage::GetData() {
  frames_.clear();
  std::ifstream infile(txt_path_);
  if (!infile.is_open()) {
    std::cerr << "Failed to open data file:" << txt_path_ << std::endl;
    return;
  }

  std::string line;
  int count = 0;
  double min_x = 99999, min_y = 99999;
  double max_x = -99999, max_y = -99999;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    std::string fp;
    int img_id, camera_id;
    double tx, ty, tz;
    double qw, qx, qy, qz;
    if (!(iss >> img_id >> qw >> qx >> qy >> qz >> tx >> ty >> tz >>
          camera_id >> fp)) {
      std::cerr << "Failed to parse line: " << line << std::endl;
      continue;
    }
    ++count;

    Eigen::Isometry3d T_enu_camera =
        GetPose(qw, qx, qy, qz, Eigen::Vector3d(tx, ty, tz));
    std::string image_name = img_path_ + fp;

    Frame f;
    f.fp = image_name;
    f.T_enu_camera = T_enu_camera;

    SetCameraFOV(f);
    min_x = std::min(f.min_pt.x(), min_x);
    min_y = std::min(f.min_pt.y(), min_y);
    max_x = std::max(f.max_pt.x(), max_x);
    max_y = std::max(f.max_pt.y(), max_y);

    frames_.push_back(f);

    std::cout << "fp: " << f.fp << std::endl;
    std::cout << "T_enu_camera :" << std::endl;
    std::cout << f.T_enu_camera.matrix() << std::endl;
    std::cout << "min_xy: " << f.min_pt.transpose() << std::endl;
    std::cout << "max_xy: " << f.max_pt.transpose() << std::endl;
  }

  enu_max_xy_ = Eigen::Vector2d(max_x, max_y);
  enu_min_xy_ = Eigen::Vector2d(min_x, min_y);
  int w = (enu_max_xy_[0] - enu_min_xy_[0]) / gsd_;
  int h = (enu_max_xy_[1] - enu_min_xy_[1]) / gsd_;
  res_w_ = (w + 255) / 256 * 256;
  res_h_ = (h + 255) / 256 * 256;
  std::cout << "width = " << res_w_ << ", height = " << res_h_ << std::endl;

  result_img_.create(res_h_, res_w_, CV_8UC3);
  for (int y = 0; y < res_h_; ++y) {
    for (int x = 0; x < res_w_; ++x) {
      result_img_.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
    }
  }

  score_layer_.create(res_h_, res_w_, CV_32F);
  for (int y = 0; y < res_h_; ++y) {
    for (int x = 0; x < res_w_; ++x) {
      score_layer_.at<float>(y, x) = -std::numeric_limits<float>::max();
    }
  }

  traj_img_.create(res_h_, res_w_, CV_8UC3);
  for (int y = 0; y < res_h_; ++y) {
    for (int x = 0; x < res_w_; ++x) {
      traj_img_.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
    }
  }
}

void OrthoImage::SetCameraFOV(Frame &frame) {
  Eigen::Vector3d camera_in_enu = frame.T_enu_camera.translation();

  // note: cancel dtm
  // double terr_h = dtm_ptr_->GetHeight(camera_in_enu[0], camera_in_enu[1]);
  // float z = std::isinf(terr_h) ? camera_in_enu[2] : camera_in_enu[2] -
  // terr_h;

  float z = camera_in_enu[2] + 100;
  Eigen::Vector3d left_up_enu, left_down_enu, right_up_enu, right_down_enu;
  UV2ENU(img_w_ / 4, img_h_ / 4, z, frame.T_enu_camera, left_up_enu);
  UV2ENU(img_w_ / 4, img_h_ * 3 / 4 - 1, z, frame.T_enu_camera, left_down_enu);
  UV2ENU(img_w_ * 3 / 4 - 1, img_h_ / 4, z, frame.T_enu_camera, right_up_enu);
  UV2ENU(img_w_ * 3 / 4 - 1, img_h_ * 3 / 4 - 1, z, frame.T_enu_camera,
         right_down_enu);

  // 计算相机的FOV边界
  BoundCorners(left_up_enu, left_down_enu, right_up_enu, right_down_enu,
               frame.min_pt, frame.max_pt);
}

void OrthoImage::UV2ENU(int u, int v, double z,
                        const Eigen::Isometry3d &T_enu_camera,
                        Eigen::Vector3d &out) {
  out = T_enu_camera * (camera_K_.inverse() * Eigen::Vector3d(u, v, 1) * z);
}

bool OrthoImage::ENU2UV(const Eigen::Isometry3d &T_enu_camera,
                        const Eigen::Vector3d &point_in_enu, int &u, int &v) {
  Eigen::Vector3d point_in_camera = T_enu_camera.inverse() * point_in_enu;
  Eigen::Vector3d point_in_image = camera_K_ * point_in_camera;
  v = std::floor(point_in_image[1] / point_in_image[2]);
  u = std::floor(point_in_image[0] / point_in_image[2]);

  if (v >= 0 && u >= 0 && v < img_h_ && u < img_w_)
    return true;
  else
    return false;
}

void OrthoImage::BoundCorners(const Eigen::Vector3d &p0,
                              const Eigen::Vector3d &p1,
                              const Eigen::Vector3d &p2,
                              const Eigen::Vector3d &p3,
                              Eigen::Vector3d &min_pt,
                              Eigen::Vector3d &max_pt) {
  min_pt[0] = std::min(p0[0], std::min(p1[0], std::min(p2[0], p3[0])));
  min_pt[1] = std::min(p0[1], std::min(p1[1], std::min(p2[1], p3[1])));
  min_pt[2] = std::min(p0[2], std::min(p1[2], std::min(p2[2], p3[2])));
  max_pt[0] = std::max(p0[0], std::max(p1[0], std::max(p2[0], p3[0])));
  max_pt[1] = std::max(p0[1], std::max(p1[1], std::max(p2[1], p3[1])));
  max_pt[2] = std::max(p0[2], std::max(p1[2], std::max(p2[2], p3[2])));
}

float OrthoImage::ComputeScore(const Eigen::Vector3d &point_in_enu,
                               const Eigen::Vector3d &camera_in_enu) {

  Eigen::Vector3d view_dir = point_in_enu - camera_in_enu;
  Eigen::Vector3d view_dir_normal = view_dir.normalized();
  Eigen::Vector3d normal(0, 0, 1);
  float score = view_dir_normal.dot(normal);
  return score;
}

void OrthoImage::Work() {
  for (const auto &f : frames_) {
    // step:读取图像
    cv::Mat image = cv::imread(f.fp);
    if (image.empty()) {
      std::cerr << "Image is empty" << std::endl;
      continue;
    }

    // note: gaussian blur to avoid aliasing
    cv::GaussianBlur(image, image, cv::Size(13, 13), 0);

    Eigen::Vector3d camera_in_enu = f.T_enu_camera.translation();
    int camera_in_enu_col = (camera_in_enu[0] - enu_min_xy_[0]) / gsd_;
    int camera_in_enu_row = (camera_in_enu[1] - enu_min_xy_[1]) / gsd_;
    cv::Point camera_pixel(camera_in_enu_col, res_h_ - 1 - camera_in_enu_row);
    cv::circle(traj_img_, camera_pixel, 2, cv::Scalar(255, 0, 255));
    std::cout << "camera_pixel:" << camera_pixel << std::endl;

    int start_col = (f.min_pt[0] - enu_min_xy_[0]) / gsd_;
    int start_row = (f.min_pt[1] - enu_min_xy_[1]) / gsd_;
    int end_col = (f.max_pt[0] - enu_min_xy_[0]) / gsd_;
    int end_row = (f.max_pt[1] - enu_min_xy_[1]) / gsd_;
    for (int y = start_row; y < end_row; y++) {
      for (int x = start_col; x < end_col; x++) {
        // note: 像素不能越界
        int res_x = x;
        int res_y = res_h_ - 1 - y;
        if (res_x >= 0 && res_x < res_w_ && res_y >= 0 && res_y < res_h_) {
          // note: cancel dtm
          // double z = dtm_ptr_->GetHeight(f.min_pt.x() + x * gsd_,
          //                                f.min_pt.y() + y * gsd_);
          // z = std::isinf(z) ? 0 : z;

          double z = -100;
          double point_in_enu_x = enu_min_xy_[0] + x * gsd_;
          double point_in_enu_y = enu_min_xy_[1] + y * gsd_;
          Eigen::Vector3d point_in_enu(point_in_enu_x, point_in_enu_y, z);
          float score = ComputeScore(point_in_enu, camera_in_enu);
          if (score > score_layer_.at<float>(res_y, res_x)) {
            int u, v;
            bool ret = ENU2UV(f.T_enu_camera, point_in_enu, u, v);
            if (ret) {
              cv::Vec3b color = image.at<cv::Vec3b>(v, u);
              score_layer_.at<float>(res_y, res_x) = score;
              // note: opencv bgr, tiff rgba
              cv::Vec3b res_color(color[0], color[1], color[2]);
              bool is_zero = (res_color[0] == 0) && (res_color[1] == 0) &&
                             (res_color[2] == 0);
              res_color = is_zero ? cv::Vec3b(1, 1, 1) : res_color;
              result_img_.at<cv::Vec3b>(res_y, res_x) = res_color;
            }
          }
        }
      }
    }

    std::cout << "Processed: " << f.fp << std::endl;
  }
  cv::imwrite("/home/zhan/res.png", result_img_);
}