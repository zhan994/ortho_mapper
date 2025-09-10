# ortho_mapper

**A simple and easy-to-use package for checking the quality of SfM through generating orthoimage.**

**This framework leverages COLMAP-based results to generate orthoimage quickly.**

<img src="example.png" style="zoom:80%;" />

## Prerequisited
- Eigen3
- OpenCV3
- PCL
- TIFF
- JSONCPP

## Example

To test this package, you can use images which has **EXIF for GPS**, use **'dev_3.10'** in [colmap](https://github.com/zhan994/colmap_detailed.git)

```bash
./work/shell/sfm_cam_gps.sh
```

Change intrinsics about camera in **'config/cfg.json'**.

```bash
mkdir build && cd build
cmake ..
make -j
./build/ortho_mapper config/cfg.json
```

