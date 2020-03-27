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

#ifndef MAP_H
#define MAP_H

#include "sfm/common.h"
#include "sfm/frame.h"
#include "sfm/mappoint.h"
#include "sfm/camera.h"

namespace sfm {

// Consists of a set of unique Frame and MapPoint.
class Map {
public:
    typedef shared_ptr<Map> Ptr;
    struct CorrData
    {
        long frameId;
        int kptIdx;
        cv::Mat P;
        cv::Vec2f kpt;
    };

public:
    Map() { camera = Camera::Ptr(new Camera); }

    void insertMapPoint(const cv::Vec3f& pos, const Track& track);
    void addObservation(long mapPointId, long frameId, int kpIdx);
    bool hasMapPoint(long id);
    bool isTwoViewObservation(Frame::Ptr frame, int kpId);

    void load();

    // setter and getter
    const unordered_map<long, Frame::Ptr> &getAllFrames() const { return frames; }
    const unordered_map<long, MapPoint::Ptr> &getAllMapPoints() const { return mapPoints; }

    // best frame selection
    vector<Frame::Ptr> findFirstInitialImage() const;
    vector<Frame::Ptr> findSecondInitialImage(Frame::Ptr firstFrame);
    vector<Frame::Ptr> findNextRegisterImage();

    void getMatchToMap(Frame::Ptr frame, vector<cv::Vec2f> &keypoint, vector<cv::Vec3f> &mapPoint, vector<int>& kpIdx, vector<long>& mapPointId);
    void getMatchToNeighbor(Frame::Ptr frame, vector<vector<CorrData>>& data);

    // save results in certain format
    void writePLYBinary(const string& path);

private:
    void loadMatches(const Database &database);
    void loadKeypoints(const Database &database);

private:
    unordered_map<long, MapPoint::Ptr> mapPoints;           // all landmarks
    unordered_map<long, Frame::Ptr> frames;                 // all frames (registered and non-registered)
    Camera::Ptr camera;
};

}

#endif // MAP_H