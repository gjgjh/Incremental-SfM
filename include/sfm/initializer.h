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

#ifndef INITIALIZER_H
#define INITIALIZER_H

#include "sfm/common.h"
#include "sfm/frame.h"

namespace sfm {

class Initializer {
public:
    typedef shared_ptr<Initializer> Ptr;

    Initializer(int maxIter=200) : maxIter(maxIter) {}
    bool initialize(Frame::Ptr referenceFrame, Frame::Ptr currentFrame);

private:
    void findHomography(vector<bool>& matchesInliers, float& score, cv::Mat &H21);
    void findFundamental(vector<bool>& matchesInliers, float& score, cv::Mat &F21);
    bool ReconstructH(const cv::Mat &H);
    bool ReconstructF(const cv::Mat &F);
    bool hasPositiveDepth(const cv::Mat& R1, const cv::Mat& t1, const cv::Mat& R2, const cv::Mat& t2);

private:
    Frame::Ptr frame1;          // reference frame
    Frame::Ptr frame2;          // current frame
    vector<cv::Point2f> kp1;    // matched keypoints from Frame1
    vector<cv::Point2f> kp2;    // matched keypoints from Frame2

    // RANSAC parameters
    int maxIter=200;
    double confidence=0.99;
    int minNumInlier=100;
    double homographyError=4.0;  // maximum allowed reprojection error to treat a point pair as an inlier
    double fundamentalError=4.0;
};

}

#endif