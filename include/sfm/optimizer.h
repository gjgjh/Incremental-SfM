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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "sfm/common.h"
#include "sfm/frame.h"
#include "sfm/mappoint.h"
#include "sfm/map.h"

namespace sfm {

class Optimizer {
public:
    typedef shared_ptr<Optimizer> Ptr;
    typedef g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType> LinearSolver;

    Optimizer();
    void globalBA(Map::Ptr map);
    void localBA(Map::Ptr map, Frame::Ptr currentFrame);

private:
    void reset();
    void bundleAdjustment(const unordered_map<long, Frame::Ptr> &frames,
                          const unordered_map<long, MapPoint::Ptr> &mappoints);
    void recoverOptimizedData(const unordered_map<long, Frame::Ptr> &frames,
                              const unordered_map<long, MapPoint::Ptr> &mappoints);
    void saveOptResult(string filepath) const;

private:
    int iteration = 10;
    int maxLocalBAFramesNum = 5;                    // max number of frames in local BA

    g2o::SparseOptimizer *optimizer = nullptr;      // g2o optimizer
    g2o::RobustKernel *robustKernel;                // robust kernel
    g2o::OptimizationAlgorithmLevenberg *solver;
    g2o::CameraParameters *camera;

    long maxFrameId = -1;
};

}

#endif