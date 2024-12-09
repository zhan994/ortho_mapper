#include "ortho.h"

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cout << "Usage: ./ortho_mapper <path-to-cfg>/cfg.json" << std::endl;
    return 1;
  }

  OrthoImage ortho_img(argv[1]);
  ortho_img.Work();
  double lt_merct_x, lt_merct_y;
  std::string tiff_path = ortho_img.GetTiff(lt_merct_x, lt_merct_y);
  std::cout << std::setprecision(10) << "Geo Info: " << lt_merct_x << " "
            << lt_merct_y << std::endl;
  return 0;
}