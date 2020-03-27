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

#include "sfm/pnpSolver.h"
#include "sfm/camera.h"

namespace sfm{


bool PnPSolver::solve(const vector<cv::Vec2f> &points2D, const vector<cv::Vec3f> &points3D, Isometry3d& T, vector<bool>& inlierMask) {
    assert(points2D.size() == points3D.size());

    cv::Mat K=Camera::instance()->K();
    cv::Mat r, t, inliers;
    cv::solvePnPRansac(points3D, points2D, K, cv::Mat(), r, t, false, maxIter, projError, confidence, inliers, method);

    int numInliers=inliers.rows;
    if(numInliers<minNumInliers) return false;

    inlierMask.clear();
    inlierMask.resize(points2D.size(), false);
    for(int i = 0; i < inliers.rows; ++i) {
        int idx = inliers.at<int>(i, 0);
        inlierMask[idx] = true;
    }

    cv::Mat R_tmp;
    cv::Rodrigues(r, R_tmp);
    Matrix3d R;
    cv2eigen(R_tmp, R);
    Vector3d tvec;
    cv2eigen(t, tvec);

    T = Isometry3d::Identity();
    AngleAxisd rvec(R);
    T.rotate(rvec);
    T.pretranslate(tvec);

    return true;
}

}