#include "ortho.h"

int main(int argc, char** argv){
  if (argc != 2)
  {
    std::cout << "Usage: ./ortho_mapper <path-to-cfg>/cfg.json" << std::endl;
    return 1;
  }

  OrthoImage ortho_img(argv[1]);
  ortho_img.Work();
  return 0;
}