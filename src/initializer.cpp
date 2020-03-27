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

#include "sfm/initializer.h"
#include "sfm/camera.h"
#include "sfm/utils.h"
#include "sfm/frame.h"

namespace sfm {

// Computes in parallel a fundamental matrix and a homography.
// Selects a model and tries to recover the motion and the structure from motion.
// Note that the reference frame is fixed.
bool Initializer::initialize(Frame::Ptr referenceFrame, Frame::Ptr currentFrame) {
    // get matched keypoints
    frame1 = referenceFrame;
    frame2 = currentFrame;
    vector<int> kpIdx1,kpIdx2;
    getMatch(frame1, frame2, kpIdx1,kpIdx2);
    for(int i=0;i<kpIdx1.size();++i){
        kp1.push_back(frame1->getKp()[kpIdx1[i]].pt);
        kp2.push_back(frame2->getKp()[kpIdx2[i]].pt);
    }

    vector<bool> matchesInliersH, matchesInliersF;
    float SH, SF;
    cv::Mat H, F;

    thread threadH(&Initializer::findHomography, this, ref(matchesInliersH), ref(SH), ref(H));
    thread threadF(&Initializer::findFundamental, this, ref(matchesInliersF), ref(SF), ref(F));

    // wait until both threads have finished
    threadH.join();
    threadF.join();

    // compute ratio of scores
    float RH = SH / SF;

    // try to reconstruct from homography or fundamental depending on the score and ratio
    if (RH > 0.7 && SH > minNumInlier) {
        cout << "Try initialize from Homography\n";
        return ReconstructH(H);
    } else if (SF > minNumInlier) {
        cout << "Try initialize from Fundamental\n";
        return ReconstructF(F);
    } else {
        cerr << "No sufficient inliers\n";
        return false;
    }
}

// Compute Homography matrix, and inlier number is used as its score.
void Initializer::findHomography(vector<bool> &matchesInliers, float &score, cv::Mat &H21) {
    cv::Mat inlierMask;
    H21 = cv::findHomography(kp1, kp2, cv::RANSAC, homographyError, inlierMask, maxIter, confidence);

    matchesInliers.resize(inlierMask.rows, false);
    score = 0;
    for (int i = 0; i < inlierMask.rows; ++i) {
        if (inlierMask.at<uchar>(i, 0) == 0) continue;

        matchesInliers[i] = true;
        ++score;
    }
}

// Compute Fundamental matrix, and inlier number is used as its score.
void Initializer::findFundamental(vector<bool> &matchesInliers, float &score, cv::Mat &F21) {
    cv::Mat inlierMask;
    F21 = cv::findFundamentalMat(kp1, kp2, cv::FM_RANSAC, fundamentalError, confidence, inlierMask);

    matchesInliers.resize(inlierMask.rows, false);
    score = 0;
    for (int i = 0; i < inlierMask.rows; ++i) {
        if (inlierMask.at<uchar>(i, 0) == 0) continue;

        matchesInliers[i] = true;
        ++score;
    }
}

// Decompose Homography matrix to obtain the pose. todo: check reprojError, parallaxAngle and numInliers
bool Initializer::ReconstructH(const cv::Mat &H) {
    cv::Mat K=Camera::instance()->K();
    vector<cv::Mat> Rs, ts;
    int solutions = cv::decomposeHomographyMat(H, K, Rs, ts, cv::noArray());

    // do chirality check
    for (int i = 0; i < solutions; i++) {
        cv::Mat R1 = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat t1 = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat R2 = Rs[i];
        cv::Mat t2 = ts[i];

        if (hasPositiveDepth(R1, t1, R2, t2)) {
            Isometry3d Tcw = Utils::matToIsometry(R2, t2);
            frame2->setTcw(Tcw);
            break;
        }
    }

    return true;
}

// Decompose Fundamental matrix to obtain the pose. todo: check reprojError, parallaxAngle and numInliers
bool Initializer::ReconstructF(const cv::Mat &F) {
    cv::Mat K=Camera::instance()->K();
    cv::Mat E = cv::findEssentialMat(kp1, kp2,K, cv::RANSAC, confidence,fundamentalError);
    cv::Mat R2, t2;
    cv::recoverPose(E, kp1, kp2, K, R2, t2); // already done chirality check

    Isometry3d Tcw = Utils::matToIsometry(R2, t2);
    frame2->setTcw(Tcw);

    return true;
}

// Positive depth constraint check, all points must be in front of the two cameras.
bool Initializer::hasPositiveDepth(const cv::Mat &R1, const cv::Mat &t1, const cv::Mat &R2, const cv::Mat &t2) {
    cv::Mat P1, P2;
    cv::Mat K=Camera::instance()->K();
    cv::hconcat(K * R1, K * t1, P1);
    cv::hconcat(K * R2, K * t2, P2);

    cv::Mat points4D;
    cv::triangulatePoints(P1, P2, kp1, kp2, points4D);

    for (int i = 0; i < points4D.rows; ++i) {
        float z = points4D.at<float>(4, i);
        if (z < 0) return false;
    }

    return true;
}

}
