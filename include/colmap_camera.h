#ifndef COLMAP_CAMERA_H
#define COLMAP_CAMERA_H

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct ColmapCamera {
  int id = 0, width = 0, height = 0;
  std::string model;
  double fx = 0, fy = 0, cx = 0, cy = 0;
};

// The mapper currently uses one shared pinhole intrinsic matrix.
inline ColmapCamera ReadColmapCamera(const std::string &path) {
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("Cannot open camera model: " + path);

  ColmapCamera camera;
  bool found = false;
  std::string line;
  int line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    std::istringstream row(line);
    row >> std::ws;
    if (row.eof() || row.peek() == '#')
      continue;
    const std::string location = path + ":" + std::to_string(line_number);
    if (found)
      throw std::runtime_error(location + ": multiple cameras are not supported");
    if (!(row >> camera.id >> camera.model >> camera.width >> camera.height) ||
        camera.id < 0 || camera.width <= 0 || camera.height <= 0)
      throw std::runtime_error(location + ": invalid camera header/dimensions");

    bool shared_focal = false;
    size_t expected = 0;
    if (camera.model == "SIMPLE_PINHOLE") {
      shared_focal = true;
      expected = 3;
    } else if (camera.model == "SIMPLE_RADIAL") {
      shared_focal = true;
      expected = 4;
    } else if (camera.model == "RADIAL") {
      shared_focal = true;
      expected = 5;
    } else if (camera.model == "PINHOLE") {
      expected = 4;
    } else if (camera.model == "OPENCV") {
      expected = 8;
    } else if (camera.model == "FULL_OPENCV") {
      expected = 12;
    } else {
      throw std::runtime_error(location + ": unsupported camera model " + camera.model);
    }

    std::vector<double> params(expected);
    for (double &param : params) {
      if (!(row >> param) || !std::isfinite(param))
        throw std::runtime_error(location + ": invalid/missing camera parameters");
    }
    row >> std::ws;
    if (!row.eof() && row.peek() != '#')
      throw std::runtime_error(location + ": unexpected camera parameters");
    camera.fx = params[0];
    camera.fy = shared_focal ? params[0] : params[1];
    camera.cx = params[shared_focal ? 1 : 2];
    camera.cy = params[shared_focal ? 2 : 3];
    if (camera.fx <= 0 || camera.fy <= 0)
      throw std::runtime_error(location + ": focal lengths must be positive");
    found = true;
  }
  if (!input.eof())
    throw std::runtime_error("Failed reading camera model: " + path);
  if (!found)
    throw std::runtime_error("No cameras in: " + path);
  return camera;
}

#endif
