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

#ifndef FRAME_H
#define FRAME_H

#include "sfm/common.h"
#include "sfm/database.h"
#include "sfm/mappoint.h"

namespace sfm {

// Frame contains everything about an image.
class Frame {
public:
    typedef shared_ptr<Frame> Ptr;
    enum class KeyPointType { SIFT, SURF };

    // ctor and factory function
    Frame(long id) : id(id) {}
    Frame(long id, const string &imageName, const cv::Mat &image) : id(id), imageName(imageName), image(image) {}
    virtual ~Frame() {}
    static Frame::Ptr createFrame(const string &imageName, const cv::Mat &image);
    static Frame::Ptr createFrame(long id);

    void detectKeypoint(KeyPointType type);
    void undistortKeypoint(const cv::Mat& K, const cv::Mat& distCoef);
    void addNumRegister() { ++numRegister; }
    void visKeyPoints() const;
    void setMapPoint(int kptIdx, MapPoint::Ptr mapPoint);
    bool hasMapPoint(int kptIdx);
    vector<long> getNeighborsId() const;

    // setter and getter
    void setMaxNumFeatures(int maxNumFeatures) { Frame::maxNumFeatures = maxNumFeatures; }
    void setMaxImageSize(int maxImageSize) { Frame::maxImageSize = maxImageSize; }
    void setKp(const vector<cv::KeyPoint> &kp) { Frame::kp = kp; }
    const vector<cv::KeyPoint> &getKp() const { return kp; }
    void setColor(const vector<cv::Vec3b> &color) { Frame::color = color; }
    const vector<cv::Vec3b> &getColor() const { return color; }
    long getId() const { return id; }
    const unordered_map<long, unordered_map<long, int>> &getMatches() const { return matches; }
    int getNumRegister() const { return numRegister; }
    const Isometry3d &getTcw() const { return Tcw; }
    void setTcw(const Isometry3d &tcw) { Tcw = tcw; }
    void setRegistered(bool registered) { Frame::registered = registered; }
    bool isRegistered() const { return registered; }
    const vector<MapPoint::Ptr> &getMapPoints() const { return mapPoints; }
    void setMapPoints(const vector<MapPoint::Ptr> &mapPoints) { Frame::mapPoints = mapPoints; }

    friend void addMatch(Frame::Ptr frame1, Frame::Ptr frame2, const vector<cv::DMatch>& match);
    friend void getMatch(Frame::Ptr frame1, Frame::Ptr frame2, vector<int>& kpIdx1, vector<int>& kpIdx2);
    friend void Database::save(const Frame* frame);

private:
    cv::Mat resizeImage();
    void selectKeypoint(const vector<cv::KeyPoint>& src,vector<cv::KeyPoint>& dst,int maxFeatureSize);
    void restoreKeypoint(const vector<cv::KeyPoint>& kpts);

private:
    // basic information
    long id;
    string imageName;

    // image information
    Isometry3d Tcw = Isometry3d::Identity();           // transform from world to camera
    cv::Mat image;
    cv::Mat desp;                                      // descriptors
    vector<cv::KeyPoint> kp;                           // keypoints
    vector<cv::Vec3b> color;                           // keypoints color
    vector<MapPoint::Ptr> mapPoints;                   // associated map points, nullptr if no association (use get())

    // match information
    // matches[idx] indicates which keypoints that keypoint idx matches on other pictures
    unordered_map<long, unordered_map<long, int>> matches;  // kptID1 : [frameID : kptID2]

    // feature extraction parameters
    int maxNumFeatures=8000;
    int maxImageSize=3200;
    double scaleWidth=1.0;
    double scaleHeight=1.0;

    // registration information
    int numRegister=0;
    bool registered=false;
};

}

#endif // FRAME_H
