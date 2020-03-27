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

#include "sfm/frame.h"

namespace sfm {

// Factory function
Frame::Ptr Frame::createFrame(const string &imageName, const cv::Mat &image) {
    static long factory_id = 0;
    return Frame::Ptr(new Frame(factory_id++, imageName, image));
}

Frame::Ptr Frame::createFrame(long id) {
    return Frame::Ptr(new Frame(id));
}

// Detect keypoint and descriptor on scaled single channel image.
void Frame::detectKeypoint(KeyPointType type) {
    cv::Mat resizedImage = resizeImage();
    cv::Mat grayImg;
    cv::cvtColor(resizedImage, grayImg, cv::COLOR_BGRA2GRAY);

    vector<cv::KeyPoint> originalKeypoint;
    vector<cv::KeyPoint> selectedKeypoint;
    if (type == KeyPointType::SIFT) {
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
        sift->detect(grayImg, originalKeypoint);

        selectKeypoint(originalKeypoint, selectedKeypoint, maxNumFeatures);
        sift->compute(grayImg, selectedKeypoint, desp);
    } else if (type == KeyPointType::SURF) {
        cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(400);
        surf->detect(grayImg, originalKeypoint);

        selectKeypoint(originalKeypoint, selectedKeypoint, maxNumFeatures);
        surf->compute(grayImg, selectedKeypoint, desp);
    }

    restoreKeypoint(selectedKeypoint);

    // L1 root normalize descriptor
    for (size_t i = 0; i < desp.rows; ++i) {
        cv::Mat row = desp.rowRange(i, i + 1);
        const double norm_l1 = cv::norm(row, cv::NORM_L1);
        row /= norm_l1;
        cv::sqrt(row, row);
    }
}

// Keypoints distortion correction.
void Frame::undistortKeypoint(const cv::Mat &K, const cv::Mat &distCoef) {
    if (distCoef.at<float>(0, 0) == 0.0) return;

    cv::Mat mat(kp.size(), 2, CV_32F);
    for (int i = 0; i < kp.size(); i++) {
        mat.at<float>(i, 0) = kp[i].pt.x;
        mat.at<float>(i, 1) = kp[i].pt.y;
    }

    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, K, distCoef, cv::Mat(), K);
    mat = mat.reshape(1);

    for (int i = 0; i < kp.size(); ++i) {
        kp[i].pt.x = mat.at<float>(i, 0);
        kp[i].pt.y = mat.at<float>(i, 1);
    }
}

void Frame::visKeyPoints() const {
    cv::Mat outimg;
    cv::drawKeypoints(image, kp, outimg, cv::Scalar(0, 255, 0));
    cv::imshow(to_string(id), outimg);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

// Set an associated map point.
void Frame::setMapPoint(int kptIdx, MapPoint::Ptr mapPoint) {
    mapPoints[kptIdx] = mapPoint;
}

// Check if has an associated map point.
bool Frame::hasMapPoint(int kptIdx) {
    if (mapPoints[kptIdx].get() == nullptr) return false;
    return true;
}

// Get all connected neighbors id.
vector<long> Frame::getNeighborsId() const {
    unordered_set<long> neighborsId;
    for (auto &id_match:matches) {
        auto &match = id_match.second;
        for (auto frameId_kpId:match) {
            long frameId = frameId_kpId.first;
            neighborsId.insert(frameId);
        }
    }

    vector<long> neighbors(neighborsId.begin(), neighborsId.end());
    return move(neighbors);
}

// Resize image so that the size won't be too large
cv::Mat Frame::resizeImage() {
    if (maxImageSize < image.rows || maxImageSize < image.cols) {
        const int width = image.cols;
        const int height = image.rows;
        const double scale = maxImageSize * 1.0 / std::max(width, height);
        const int new_width = width * scale;
        const int new_height = height * scale;

        scaleWidth = new_width * 1.0 / width;
        scaleHeight = new_height * 1.0 / height;

        cv::Mat resized;
        cv::resize(image, resized, cv::Size(new_width, new_height));
        return resized;
    } else {
        return image;
    }
}

// Select maxFeatureSize keypoints with the largest size
void Frame::selectKeypoint(const vector<cv::KeyPoint> &src, vector<cv::KeyPoint> &dst, int maxFeatureSize) {
    if (maxFeatureSize > src.size()) {
        dst = src;
    } else {
        std::vector<std::pair<size_t, float>> scales;
        for (size_t i = 0; i < src.size(); ++i)
            scales.emplace_back(i, src[i].size);

        std::partial_sort(scales.begin(), scales.begin() + maxFeatureSize,
                          scales.end(),
                          [](const std::pair<size_t, float> scale1, const std::pair<size_t, float> scale2) {
                              return scale1.second > scale2.second;
                          }
        );

        dst.reserve(maxFeatureSize);
        for (size_t i = 0; i < maxFeatureSize; ++i) {
            dst.push_back(src[scales[i].first]);
        }
    }
}

// Convert keypoints back to original coordinates (before downsampling)
void Frame::restoreKeypoint(const vector<cv::KeyPoint> &kpts) {
    const double inv_scale_x = 1.0 / scaleWidth;
    const double inv_scale_y = 1.0 / scaleHeight;
    const double inv_scale_xy = (inv_scale_x + inv_scale_y) / 2.0f;

    kp.resize(kpts.size());
    color.resize(kpts.size());
    for (size_t i = 0; i < kpts.size(); ++i) {
        kp[i].pt.x = kpts[i].pt.x * inv_scale_x;
        kp[i].pt.y = kpts[i].pt.y * inv_scale_y;
        kp[i].size = kpts[i].size * inv_scale_xy;
        kp[i].angle = kpts[i].angle;

        color[i] = image.at<cv::Vec3b>(kp[i].pt.y, kp[i].pt.x);
    }
}

// Add match between two frames.
void addMatch(Frame::Ptr frame1, Frame::Ptr frame2, const vector<cv::DMatch> &match) {
    for (auto &m:match) {
        int queryIdx = m.queryIdx;
        int trainIdx = m.trainIdx;
        frame1->matches[queryIdx][frame2->id] = trainIdx;
        frame2->matches[trainIdx][frame1->id] = queryIdx;
    }
}

// Get matched keypoints between two frames.
void getMatch(Frame::Ptr frame1, Frame::Ptr frame2, vector<int> &kpIdx1, vector<int> &kpIdx2) {
    vector<int> keypointIdx1;
    vector<int> keypointIdx2;
    auto &id_matches = frame1->getMatches();
    int id2 = frame2->getId();
    for (auto &id_match:id_matches) {
        auto &match = id_match.second;
        if (match.find(id2) != match.end()) {
            int idx1 = id_match.first;
            int idx2 = match.at(id2);
            keypointIdx1.push_back(idx1);
            keypointIdx2.push_back(idx2);
        }
    }

    kpIdx1 = move(keypointIdx1);
    kpIdx2 = move(keypointIdx2);
}

}

