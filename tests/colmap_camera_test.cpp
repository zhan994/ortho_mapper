#include "colmap_camera.h"

#include <filesystem>
#include <iostream>

int main() {
  const auto dir = std::filesystem::temp_directory_path() /
                   ("ortho-camera-test-" + std::to_string(
                       std::filesystem::file_time_type::clock::now().time_since_epoch().count()));
  std::filesystem::create_directory(dir);
  const auto path = (dir / "cameras.txt").string();
  const auto write = [&](const std::string &text) {
    std::ofstream out(path);
    out << text;
  };
  const auto check = [](bool ok) {
    if (!ok)
      throw std::runtime_error("Camera test failed");
  };
  const auto reject = [&](const std::string &text) {
    write(text);
    bool rejected = false;
    try {
      ReadColmapCamera(path);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    check(rejected);
  };
  try {
    write("# cameras\n \n  # comment\n1 OPENCV 3000 4000 3200 3300 1500 2000 -.1 .02 .001 -.002\n");
    auto camera = ReadColmapCamera(path);
    check(camera.id == 1 && camera.width == 3000 && camera.height == 4000 &&
          camera.fx == 3200 && camera.fy == 3300 && camera.cx == 1500 && camera.cy == 2000);
    for (const std::string &model : {"SIMPLE_PINHOLE", "SIMPLE_RADIAL", "RADIAL"}) {
      write("2 " + model + " 640 480 500 320 240" +
            (model == "SIMPLE_PINHOLE" ? "" : model == "RADIAL" ? " .1 .2" : " .1"));
      camera = ReadColmapCamera(path);
      check(camera.fx == 500 && camera.fy == 500 && camera.cx == 320 && camera.cy == 240);
    }
    write("1 PINHOLE 640 480 500 510 320 240");
    check(ReadColmapCamera(path).fy == 510);
    write("1 FULL_OPENCV 640 480 500 510 320 240 0 0 0 0 0 0 0 0");
    check(ReadColmapCamera(path).cy == 240);
    reject("# empty\n");
    reject("1 UNKNOWN 640 480 500 510 320 240");
    reject("1 OPENCV 640 480 500 510 320 240");
    reject("1 PINHOLE 0 480 500 510 320 240");
    reject("1 PINHOLE 640 480 -1 510 320 240");
    reject("1 PINHOLE 640 480 nan 510 320 240");
    reject("1 PINHOLE 640 480 500 510 320 240 extra");
    reject("1 PINHOLE 640 480 500 510 320 240\n2 PINHOLE 640 480 500 510 320 240\n");
    std::filesystem::remove(path);
    bool missing = false;
    try {
      ReadColmapCamera(path);
    } catch (const std::runtime_error &) {
      missing = true;
    }
    check(missing);
    std::filesystem::remove(dir);
    std::cout << "Camera parser tests passed\n";
  } catch (...) {
    std::filesystem::remove_all(dir);
    throw;
  }
}
