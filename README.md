# ortho_mapper

**A repo. which used to easily check the quality of SfM through generating orthoimage.**

## 3rdparty
- Eigen3
- OpenCV3
- PCL

## run

For images which has EXIF for GPS, use 'dev_3.10' in [colmap](https://github.com/zhan994/colmap_detailed.git)

```bash
./work/shell/sfm_cam_gps.sh
```

Change intrinsics about camera in 'config/cfg.json'.

```bash
mkdir build && cd build
cmake ..
make -j
./build/ortho_mapper config/cfg.json
```

