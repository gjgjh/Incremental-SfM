# Incremental Sfm

This program is a simple Incremental Sfm (Structure from Motion) project. It uses sequential or unordered images as inputs, then restores camera poses and reconstructs 3D point cloud information of scene. I finished and improved this program based on the [MonocularSfM](https://github.com/nebula-beta/MonocularSfM) and [colmap](https://github.com/colmap/colmap), thanks for their great work!

Similar to [ORBSLAM](https://github.com/raulmur/ORB_SLAM2), the initialization process considers both planar and non-planar cases. Then this program uses PnP algorithm in registering (front end) and bundle adjustment optimization in the back end. This program save results in .ply format and you can visualize the point cloud in Meshlab, Colmap or other software.

Note that this code is relatively simple, mainly for learning use. I will optimize this program in the future to improve the accuracy.

![](https://tva1.sinaimg.cn/large/00831rSTgy1gdfj6jodzrg30uk0m8npi.gif)

## 1 Prerequisites

- C++11 Compiler. Some functionalities of C++11 are used.

- [OpenCV](https://github.com/opencv/opencv). Because sift or surf feature points are required, additional contrib modules need to be installed.
- [Eigen3](http://eigen.tuxfamily.org/). A C++ template library for linear algebra.
- [g2o](https://github.com/RainerKuemmerle/g2o). A library for nonlinear optimization, like bundle adjustment.

## 2 Installation

Clone the repository and make:

```bash
git clone https://github.com/gjgjh/Incremental-SfM
cd Incremental-SfM
mkdir build
cd build
cmake ..
make -j4
```

## 3 Usage

Before running the program, you may need to adjust some parameters setting in config/custom_config.yaml. Some famous [datasets](https://colmap.github.io/datasets.html#datasets) config files are also provided. For detailed information, you can look into the comment.

Then for example, simply run these from the command line:

```bash
bin/FeatureExtraction config/south-building.yaml
bin/ComputeMatches config/south-building.yaml
bin/Reconstruction config/south-building.yaml
```

## 4 License

The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues or commercial inquiries, please contact GJH <guojh_rs@pku.edu.cn>.