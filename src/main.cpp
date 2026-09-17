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
  std::string dsm_path = ortho_img.GetDSM();
  float min_height, max_height;
  std::string dsm_vis_path =
      ortho_img.GetDSMVisualization(min_height, max_height);
  std::cout << std::setprecision(10) << "Geo Info: " << lt_merct_x << " "
            << lt_merct_y << std::endl;
  std::cout << "DOM: " << tiff_path << std::endl;
  std::cout << "DSM: " << dsm_path << std::endl;
  std::cout << "DSM visualization: " << dsm_vis_path << std::endl;
  if (!dsm_vis_path.empty())
    std::cout << "DSM altitude range: " << min_height << " - " << max_height
              << " m" << std::endl;
  else
    std::cout << "DSM altitude range: unavailable" << std::endl;
  return 0;
}
