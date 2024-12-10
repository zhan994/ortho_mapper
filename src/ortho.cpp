#include "ortho.h"

OrthoImage::OrthoImage(const std::string &cfg) : map_pcl_enu_(new PointCloud) {
  std::cout << "Cons. OrthoImage: " << cfg << std::endl;
  terr_ptr_ = std::make_shared<Terr>();

  if (LoadCfg(cfg))
    std::cout << "Load cfg done." << std::endl;

  GetData();
  InitTiff();

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

    img_w_ = value["img_size"][0].asInt();
    img_h_ = value["img_size"][1].asInt();

    double fx = value["intrinsics"][0].asDouble();
    double fy = value["intrinsics"][1].asDouble();
    double cx = value["intrinsics"][2].asDouble();
    double cy = value["intrinsics"][3].asDouble();
    camera_K_ << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    gsd_ = value["gsd"].asDouble();
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
  std::ifstream infile(txt_path_);
  if (!infile.is_open()) {
    std::cerr << "Failed to open data file:" << txt_path_ << std::endl;
    return;
  }

  // step: 1 解析 txt文件获取 ECEF坐标系下的信息
  std::vector<Eigen::Vector3d> pos_ecef_vec;
  std::vector<Eigen::Quaterniond> rot_ecef_vec;
  std::vector<std::string> img_name_vec;
  std::vector<int> img_id_vec;
  std::vector<Eigen::Vector3d> pts_ecef_vec;
  int count = 0;
  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    if (line.substr(0, 5) == "POINT") {
      std::string fn;
      double px, py, pz;
      if ((iss >> fn >> px >> py >> pz)) {
        Eigen::Vector3d pt_ecef(px, py, pz);
        pts_ecef_vec.push_back(pt_ecef);
      } else {
        std::cout << "Error: Failed to parse result line: " << line
                  << std::endl;
      }
    } else {
      std::string fn;
      int img_id, camera_id;
      double tx, ty, tz;
      double qw, qx, qy, qz;
      if ((iss >> img_id >> qw >> qx >> qy >> qz >> tx >> ty >> tz >>
           camera_id >> fn)) {
        ++count;
        std::string image_name = img_path_ + fn;
        Eigen::Quaterniond q_ecef(qw, qx, qy, qz);
        Eigen::Vector3d pos_ecef(tx, ty, tz);
        rot_ecef_vec.push_back(q_ecef);
        pos_ecef_vec.push_back(pos_ecef);
        img_name_vec.push_back(image_name);
        img_id_vec.push_back(img_id);
      } else {
        std::cout << "Error: Failed to parse result line: " << line
                  << std::endl;
      }
    }
  }

  // step: 2 ECEF 转 ELL
  std::vector<Eigen::Vector3d> pos_ell_vec = gps_tform_.XYZToEll(pos_ecef_vec);
  ori_ecef_ = pos_ecef_vec[0];
  ori_ell_ = pos_ell_vec[0];
  ori_merct_ = gps_tform_.XYZToMercator({ori_ecef_})[0];

  // step: 3 ECEF 转 ENU
  std::vector<Eigen::Vector3d> pos_enu_vec =
      gps_tform_.XYZToENU(pos_ecef_vec, ori_ell_[0], ori_ell_[1]);
  ori_enu_ = pos_enu_vec[0];

  std::vector<Eigen::Quaterniond> rot_enu_vec =
      gps_tform_.XYZToENURotation(rot_ecef_vec, ori_ell_[0], ori_ell_[1]);
  frames_.clear();
  for (int i = 0; i < count; ++i) {
    Frame frame;
    frame.img_name = img_name_vec[i];
    frame.img_id = img_id_vec[i];
    frame.pos_ecef = pos_ecef_vec[i];
    frame.rot_ecef = rot_ecef_vec[i];
    frame.pos_ell = pos_ell_vec[i];
    frame.pos_enu = pos_enu_vec[i];
    frame.rot_enu = rot_enu_vec[i];
    frame.T_enu_camera = GetPose(frame.rot_enu, frame.pos_enu);
    // std::cout << std::setprecision(10) << "Frame " << i << ": "
    //           << frame.img_name << ", id: " << frame.img_id
    //           << ", pos_ecef: " << frame.pos_ecef.transpose()
    //           << ", pos_enu: " << frame.pos_enu.transpose()
    //           << ", pos_ell: " << frame.pos_ell.transpose() << std::endl;
    frames_.push_back(frame);
  }

  pts_ecef_vec.insert(pts_ecef_vec.begin(), pos_ecef_vec[0]);
  std::vector<Eigen::Vector3d> pts_enu_vec =
      gps_tform_.XYZToENU(pts_ecef_vec, ori_ell_[0], ori_ell_[1]);
  for (int i = 1; i < pts_enu_vec.size(); ++i) {
    PointT pt;
    pt.x = pts_enu_vec[i][0];
    pt.y = pts_enu_vec[i][1];
    pt.z = pts_enu_vec[i][2];
    pt.intensity = 255;
    map_pcl_enu_->points.push_back(pt);
  }
  map_pcl_enu_->width = map_pcl_enu_->points.size();
  map_pcl_enu_->height = 1;
  pcl::io::savePLYFile("map_pcl_enu.ply", *map_pcl_enu_);
  terr_ptr_->SetData(map_pcl_enu_, grid_size_, percent_, ct_kd_interp_);

  std::cout << "========> Ori camera position: " << std::endl;
  std::cout << "enu: " << ori_enu_.transpose() << std::endl;
  std::cout << "ell: " << ori_ell_.transpose() << std::endl;
  std::cout << "ecef:" << ori_ecef_.transpose() << std::endl;
  std::cout << "merct:" << ori_merct_.transpose() << std::endl;
  std::cout << "========> Ori camera position. " << std::endl;
}

void OrthoImage::InitTiff() {
  std::cout << "Frames Size: " << frames_.size() << std::endl;

  double min_x = 99999, min_y = 99999;
  double max_x = -99999, max_y = -99999;
  for (int i = 0; i < frames_.size(); ++i) {
    SetCameraFOV(frames_[i]);
    min_x = std::min(frames_[i].min_pt.x(), min_x);
    min_y = std::min(frames_[i].min_pt.y(), min_y);
    max_x = std::max(frames_[i].max_pt.x(), max_x);
    max_y = std::max(frames_[i].max_pt.y(), max_y);
  }

  enu_max_ = Eigen::Vector3d(max_x, max_y, 0);
  enu_min_ = Eigen::Vector3d(min_x, min_y, 0);
  merct_max_ = gps_tform_.ENUToMercator({enu_max_}, ori_ell_[0], ori_ell_[1],
                                        ori_ell_[2])[0];
  merct_min_ = gps_tform_.ENUToMercator({enu_min_}, ori_ell_[0], ori_ell_[1],
                                        ori_ell_[2])[0];
  // std::cout << "enu_max_: " << enu_max_.transpose() << std::endl;
  // std::cout << "enu_min_: " << enu_min_.transpose() << std::endl;
  // std::cout << "merct_max_: " << merct_max_.transpose() << std::endl;
  // std::cout << "merct_min_: " << merct_min_.transpose() << std::endl;

  int merct_width = (merct_max_[0] - merct_min_[0]) / gsd_;
  int merct_height = (merct_max_[1] - merct_min_[1]) / gsd_;
  std::cout << "merct_width = " << merct_width
            << ", merct_height = " << merct_height << std::endl;

  result_width_ = (merct_width + 255) / 256 * 256;
  result_height_ = (merct_height + 255) / 256 * 256;
  std::cout << "width = " << result_width_ << ", height = " << result_height_
            << std::endl;
  result_img_.create(result_height_, result_width_, CV_8UC3);
  for (int y = 0; y < result_height_; ++y) {
    for (int x = 0; x < result_width_; ++x) {
      result_img_.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
    }
  }

  score_layer_.create(result_height_, result_width_, CV_32F);
  for (int y = 0; y < result_height_; ++y) {
    for (int x = 0; x < result_width_; ++x) {
      score_layer_.at<float>(y, x) = -std::numeric_limits<float>::max();
    }
  }

  traj_img_.create(result_height_, result_width_, CV_8UC3);
  for (int y = 0; y < result_height_; ++y) {
    for (int x = 0; x < result_width_; ++x) {
      traj_img_.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
    }
  }
}

void OrthoImage::SetCameraFOV(Frame &frame) {
  Eigen::Vector3d camera_in_enu = frame.T_enu_camera.translation();

  double terr_h = terr_ptr_->GetHeight(camera_in_enu[0], camera_in_enu[1]);
  if (std::isinf(terr_h)) {
    std::cout << "terr_h is inf" << std::endl;
    return;
  }

  double z = camera_in_enu[2] - terr_h;
  Eigen::Vector3d left_up_enu, left_down_enu, right_up_enu, right_down_enu;
  UV2ENU(img_w_ / 4, img_h_ / 4, z, frame.T_enu_camera, left_up_enu);
  UV2ENU(img_w_ / 4, img_h_ * 3 / 4 - 1, z, frame.T_enu_camera, left_down_enu);
  UV2ENU(img_w_ * 3 / 4 - 1, img_h_ / 4, z, frame.T_enu_camera, right_up_enu);
  UV2ENU(img_w_ * 3 / 4 - 1, img_h_ * 3 / 4 - 1, z, frame.T_enu_camera,
         right_down_enu);

  // 计算相机的FOV边界
  BoundCorners(left_up_enu, left_down_enu, right_up_enu, right_down_enu,
               frame.min_pt, frame.max_pt);
  // std::cout << "min_pt: " << frame.min_pt.transpose() << std::endl;
  // std::cout << "max_pt: " << frame.max_pt.transpose() << std::endl;
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

  if (v >= img_h_ / 4 && u >= img_w_ / 4 && v < img_h_ * 3 / 4 &&
      u < img_w_ * 3 / 4)
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
    cv::Mat image = cv::imread(f.img_name);
    if (image.empty()) {
      std::cerr << "Image is empty" << std::endl;
      continue;
    }

    // note: gaussian blur to avoid aliasing
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0);

    Eigen::Vector3d f_merct_min = gps_tform_.ENUToMercator(
        {f.min_pt}, ori_ell_[0], ori_ell_[1], ori_ell_[2])[0];
    int start_col = (f_merct_min[0] - merct_min_[0]) / gsd_;
    int start_row = (f_merct_min[1] - merct_min_[1]) / gsd_;

    Eigen::Vector3d f_merct_max = gps_tform_.ENUToMercator(
        {f.max_pt}, ori_ell_[0], ori_ell_[1], ori_ell_[2])[0];
    int end_col = (f_merct_max[0] - merct_min_[0]) / gsd_;
    int end_row = (f_merct_max[1] - merct_min_[1]) / gsd_;

    Eigen::Vector3d camera_in_enu = f.pos_enu;
    Eigen::Vector3d camera_in_merct = gps_tform_.ENUToMercator(
        {camera_in_enu}, ori_ell_[0], ori_ell_[1], ori_ell_[2])[0];
    int camera_in_merct_col = (camera_in_merct[0] - merct_min_[0]) / gsd_;
    int camera_in_merct_row = (camera_in_merct[1] - merct_min_[1]) / gsd_;
    cv::Point camera_pixel(camera_in_merct_col,
                           result_height_ - 1 - camera_in_merct_row);
    cv::circle(traj_img_, camera_pixel, 2, cv::Scalar(255, 0, 255));
    std::cout << "camera_pixel:" << camera_pixel << std::endl;

    for (int y = start_row; y < end_row; y++) {
      for (int x = start_col; x < end_col; x++) {
        // note: 像素不能越界
        int res_x = x;
        int res_y = result_height_ - 1 - y;
        if (res_x >= 0 && res_x < result_width_ && res_y >= 0 &&
            res_y < result_height_) {
          Eigen::Vector3d point_in_merct =
              merct_min_ + Eigen::Vector3d(x * gsd_, y * gsd_, 0);
          Eigen::Vector3d point_in_enu = gps_tform_.MercatorToENU(
              {ori_merct_, point_in_merct}, ori_ell_[0], ori_ell_[1])[1];

          double z = terr_ptr_->GetHeight(point_in_enu[0], point_in_enu[1]);
          if (std::isinf(z)) {
            std::cout << "z is inf" << std::endl;
            continue;
          }
          point_in_enu[2] = z;
          float score = ComputeScore(point_in_enu, camera_in_enu);
          if (score > score_layer_.at<float>(res_y, res_x)) {
            int u, v;
            bool ret = ENU2UV(f.T_enu_camera, point_in_enu, u, v);
            if (ret) {
              cv::Vec3b color = image.at<cv::Vec3b>(v, u);
              score_layer_.at<float>(res_y, res_x) = score;
              // note: opencv bgr, tiff rgba
              cv::Vec3b res_color(color[2], color[1], color[0]);
              bool is_zero = (res_color[0] == 0) && (res_color[1] == 0) &&
                             (res_color[2] == 0);
              res_color = is_zero ? cv::Vec3b(1, 1, 1) : res_color;
              result_img_.at<cv::Vec3b>(res_y, res_x) = res_color;
            }
          }
        }
      }
    }
    std::cout << "Processed: " << f.img_name << std::endl;
  }
  // cv::imwrite("/home/zhan/res.png", traj_img_);
}

std::string OrthoImage::GetTiff(double &lt_merct_x, double &lt_merct_y) {
  std::cout << "Save Tiff,,," << std::endl;

  cv::Mat merged;
  AddTraj(merged);

  CorrectTiff(1.5);

  std::string file_name = "result.tif";
  std::string traj_name = "traj.png";
  TIFF *out = TIFFOpen(file_name.c_str(), "w");
  if (!out) {
    return std::string();
  }
  TIFFSetField(out, TIFFTAG_IMAGEWIDTH, result_img_.cols);
  TIFFSetField(out, TIFFTAG_IMAGELENGTH, result_img_.rows);
  TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, 3);
  TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, 8);
  TIFFSetField(out, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
  TIFFSetField(out, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(out, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
  TIFFSetField(out, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);

  tsize_t linebytes = result_img_.cols * 3 * sizeof(uchar);
  unsigned char *buf = nullptr;
  if (TIFFScanlineSize(out) == linebytes)
    buf = (unsigned char *)_TIFFmalloc(linebytes);
  else
    buf = (unsigned char *)_TIFFmalloc(TIFFScanlineSize(out));

  TIFFSetField(out, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(out, linebytes));
  for (size_t row = 0; row < result_img_.rows; row++) {
    memcpy(buf, result_img_.ptr(row), linebytes);
    if (TIFFWriteScanline(out, buf, row, 0) < 0)
      break;
  }

  TIFFClose(out);
  if (buf)
    _TIFFfree(buf);

  // left-top merct
  lt_merct_x = merct_min_[0];
  lt_merct_y = merct_min_[1] + gsd_ * result_height_;

  cv::imwrite(traj_name, merged);

  return file_name;
}

void OrthoImage::AddTraj(cv::Mat &merged) {
  cv::Mat hsv_overlay;
  cv::cvtColor(traj_img_, hsv_overlay, cv::COLOR_BGR2HSV);

  // 定义黑色的阈值范围
  cv::Scalar lower_black(0, 0, 0);
  cv::Scalar upper_black(180, 255, 30);

  // 创建掩码，非黑色的部分为白色，黑色的部分为黑色
  cv::Mat mask;
  cv::inRange(hsv_overlay, lower_black, upper_black, mask);

  // 取反掩码，非黑色的部分为黑色，黑色的部分为白色
  cv::Mat mask_inv;
  cv::bitwise_not(mask, mask_inv);

  // 覆盖区域的前景
  cv::Mat overlay_fg;
  cv::bitwise_and(traj_img_, traj_img_, overlay_fg, mask_inv);

  // 背景区域的前景
  cv::Mat cvt_result;
  cv::cvtColor(result_img_, cvt_result, cv::COLOR_RGB2BGR);
  cv::Mat background_fg;
  cv::bitwise_and(cvt_result, cvt_result, background_fg, mask);

  // 合并前景
  merged.release();
  cv::add(background_fg, overlay_fg, merged);
}

void OrthoImage::CorrectTiff(double gamma) {
  cv::Mat lut(1, 256, CV_8U);
  uchar *p = lut.ptr();
  for (int i = 0; i < 256; ++i) {
    p[i] = cv::saturate_cast<uchar>(std::pow(i / 255.0, gamma) * 255.0);
  }
  cv::LUT(result_img_, lut, result_img_);
}