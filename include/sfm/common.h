/*******************************************************
 * Copyright (C) 2020, Navigation And Location Group, Peking University
 * 
 * This file is part of IncrementalSfm.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: GJH (guojh_rs@pku.edu.cn)
 *******************************************************/

#ifndef COMMON_H
#define COMMON_H

// define the commonly included file to avoid a long include list
// for std
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <thread>
using namespace std;

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Isometry3d;
using Eigen::AngleAxisd;

// for opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// for g2o
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

#endif // COMMON_H