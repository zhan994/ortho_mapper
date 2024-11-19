# ortho_mapper

**A repo. which used to easily check the quality of SfM through generating orthoimage.**

## 3rdparty
- Eigen3
- OpenCV3

## run

For images which has EXIF for GPS, use 'dev_3.10' in [Colmap](https://github.com/zhan994/colmap_detailed.git)

```bash
./scripts/shell/sfm.sh
./scripts/shell/gps_align_db.sh
python3 scripts/python/colmap_pose.py proj/sparse/0_aligned_enu/images.txt proj/sparse/0_aligned_enu/images_Twc.txt
python3 scripts/python/export_sfm_ply.py proj/sparse/0_aligned_enu/points3D.txt proj/points3D.ply
```

Change intrinsics about camera in 'config/cfg.json'.

```bash
mkdir build && cd build
cmake ..
make -j
./build/ortho_mapper config/cfg.json
```

