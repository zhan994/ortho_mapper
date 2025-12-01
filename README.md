# ortho_mapper

**The repository contains the source code for ortho_mapper module of our system. This partial release aims to provide insight into the underlying implementation of our approach and may be useful for researchers working in related areas.**

**This is a simple and easy-to-use package for checking the quality of SfM through generating orthoimage quickly.**

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

## Citation

```
@INPROCEEDINGS{11247677,
  author={He, Jialei and Zhan, Zhihao and Tu, Zhituo and Zhu, Xiang and Yuan, Jie},
  booktitle={2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={A Multi-Sensor Fusion Approach for Rapid Orthoimage Generation in Large-Scale UAV Mapping}, 
  year={2025},
  volume={},
  number={},
  pages={6808-6815},
  keywords={Visualization;Accuracy;Structure from motion;Robot vision systems;Sensor fusion;Autonomous aerial vehicles;Cameras;Robustness;Sensors;Global Positioning System},
  doi={10.1109/IROS60139.2025.11247677}}

```
