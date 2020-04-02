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

#include "sfm/triangulator.h"
#include "sfm/frame.h"
#include "sfm/camera.h"

namespace sfm {

// Triangulate 2 or N views for multiple 3D points.
// points2d is vector of vectors of 2d points (the inner vector is per image);
// Ps is vector with 3x4 projections matrices of each image;
// points3d is output array with computed 3d points.
void Triangulator::triangulate(const vector<vector<cv::Point2f>> &points2d, const vector<cv::Mat> &Ps,
                               vector<cv::Point3f> &points3d) {
    // check
    int numViews = points2d.size();
    assert(numViews >= 2 && numViews == Ps.size());

    int numPoints2d = points2d[0].size();
    points3d.clear();

    for (int i = 0; i < numPoints2d; ++i) {
        vector<cv::Point2f> temp;
        for (int j = 0; j < numViews; ++j) {
            temp.push_back(points2d[j][i]);
        }

        cv::Point3f pts3d;
        triangulateMultiviews(temp, Ps, pts3d);
        points3d.push_back(pts3d);
    }
}

// Triangulate 2 frames for their matched points.
void Triangulator::triangulateTwoFrames(Frame::Ptr frame1, Frame::Ptr frame2, vector<cv::Point3f> &points3d) {
    cv::Mat P1, P2;
    cv::eigen2cv(frame1->getTcw().matrix(), P1);
    cv::eigen2cv(frame2->getTcw().matrix(), P2);

    // P=K[R|t]
    cv::Mat K=Camera::instance()->K();
    P1 = K * P1.rowRange(0, 3);
    P2 = K * P2.rowRange(0, 3);

    vector<cv::Mat> Ps = {P1, P2};

    vector<int> kpIdx1, kpIdx2;
    getMatch(frame1, frame2, kpIdx1, kpIdx2);

    vector<cv::Point2f> kp1, kp2;
    for (int i = 0; i < kpIdx1.size(); ++i) {
        kp1.push_back(frame1->getKp()[kpIdx1[i]].pt);
        kp2.push_back(frame2->getKp()[kpIdx2[i]].pt);
    }
    vector<vector<cv::Point2f>> match{kp1, kp2};

    triangulate(match, Ps, points3d);
}

// Triangulate 2 or N views for one 3D point.
void Triangulator::triangulateMultiviews(const vector<cv::Point2f> &points2d, const vector<cv::Mat> &Ps,
                                         cv::Point3f &point3d) {
    // check
    int numViews = points2d.size();
    assert(numViews >= 2 && numViews == Ps.size());

    // solve Ax=0, x is the eigenvector of ATA corresponding to the smallest eigenvalue
    cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
    for (size_t i = 0; i < numViews; ++i) {
        cv::Mat P = Ps[i];

        const cv::Mat term1 = points2d[i].x * P.row(2) - P.row(0);
        const cv::Mat term2 = points2d[i].y * P.row(2) - P.row(1);

        A += term1.t() * term1;
        A += term2.t() * term2;
    }

    cv::Mat eigenvalues;
    cv::Mat eigenvector;
    cv::eigen(A, eigenvalues, eigenvector);

    assert(eigenvector.type() == CV_64F);

    double x = eigenvector.at<double>(3, 0) / eigenvector.at<double>(3, 3);
    double y = eigenvector.at<double>(3, 1) / eigenvector.at<double>(3, 3);
    double z = eigenvector.at<double>(3, 2) / eigenvector.at<double>(3, 3);

    point3d.x = x;
    point3d.y = y;
    point3d.z = z;
}

}