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

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include "sfm/common.h"

namespace sfm {

class PnPSolver {
public:
    typedef shared_ptr<PnPSolver> Ptr;

    bool solve(const vector<cv::Vec2f> &points2D, const vector<cv::Vec3f> &points3D, Isometry3d &T, vector<bool> &inlierMask);

private:
    int minNumInliers = 10;

    // some RANSAC parameters
    int maxIter = 100;
    float projError = 8.0;
    double confidence = 0.99;

    cv::SolvePnPMethod method = cv::SOLVEPNP_EPNP;
};

}

#endif