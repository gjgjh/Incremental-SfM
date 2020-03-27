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

#include "sfm/reconstruction.h"
#include "sfm/mappoint.h"
#include "sfm/config.h"

namespace sfm {

Reconstruction::Reconstruction() {
    initializer = Initializer::Ptr(new Initializer);
    optimizer = Optimizer::Ptr(new Optimizer);
    triangulator = Triangulator::Ptr(new Triangulator);
    pnpSolver = PnPSolver::Ptr(new PnPSolver);

    localOptInterval = Config::instance()->get<int>("localOptInterval");
    globalOptRatio = Config::instance()->get<float>("globalOptRatio");

    map = Map::Ptr(new Map);
    map->load();
}

void Reconstruction::run() {
    // initialize
    bool isInitialized = tryInitialize(50);
    if (!isInitialized) {
        cerr << "Initialize failed\n";
        return;
    }
    numRegistered = 2;
    optimizer->globalBA(map);

    // register and optimize
    while (true) {
        vector<Frame::Ptr> nextFrames = map->findNextRegisterImage();
        int numNextFrames = nextFrames.size();
        if (numNextFrames == 0) break;

        int numFailure = 0;
        for (auto &frame:nextFrames) {
            bool success = registerNext(frame);
            if (success) {
                ++numRegistered;
                if (numRegistered > lastNumRegistered * globalOptRatio) {
                    optimizer->globalBA(map);
                    lastNumRegistered = numRegistered;
                } else if (numRegistered % localOptInterval == 0) {
                    optimizer->localBA(map, frame);
                }
            } else {
                ++numFailure;
                continue;
            }
        }

        if (numFailure == numNextFrames) break;
    }

    optimizer->globalBA(map);
}

// Save results.
void Reconstruction::writePLYBinary(const string &path) {
    map->writePLYBinary(path);
}

// Try to find a good image pair to initialize, up to maxTrial times.
bool Reconstruction::tryInitialize(int maxTrial) {
    int numTrial = 0;
    vector<Frame::Ptr> firstFrameCandidates = map->findFirstInitialImage();
    for (auto &firstFrame:firstFrameCandidates) {
        vector<Frame::Ptr> secondFrameCandidates = map->findSecondInitialImage(firstFrame);
        for (auto &secondFrame:secondFrameCandidates) {
            int id1 = firstFrame->getId();
            int id2 = secondFrame->getId();
            cout << "Try intialize: " << id1 << " , " << id2 << '\n';
            ++numTrial;
            firstFrame->addNumRegister();
            secondFrame->addNumRegister();

            bool isInitialized = initializer->initialize(firstFrame, secondFrame);

            if (isInitialized) {
                cout << "Initialize succeed " << id1 << " , " << id2 << '\n';
                firstFrame->setRegistered(true);
                secondFrame->setRegistered(true);

                // triangulate and update map
                vector<cv::Point3f> points3d;
                triangulator->triangulateTwoFrames(firstFrame, secondFrame, points3d);
                vector<int> kpIdx1, kpIdx2;
                getMatch(firstFrame, secondFrame, kpIdx1, kpIdx2);
                for (int i = 0; i < points3d.size(); ++i) {
                    Track track;
                    track.addElement(id1, kpIdx1[i]);
                    track.addElement(id2, kpIdx2[i]);
                    map->insertMapPoint(points3d[i], track);
                }

                return true;
            } else {
                if (numTrial > maxTrial) return false;
            }
        }
    }

    return false;
}

// Try to register a frame.
bool Reconstruction::registerNext(Frame::Ptr frame) {
    int id = frame->getId();
    frame->addNumRegister();
    cout << "Try register: " << id << '\n';

    vector<cv::Vec2f> kp;
    vector<cv::Vec3f> mapPoint;
    vector<int> kpIdx;
    vector<long> mapPointId;
    map->getMatchToMap(frame, kp, mapPoint, kpIdx, mapPointId);

    Isometry3d T;
    vector<bool> inlierMask;
    bool isRegistered = pnpSolver->solve(kp, mapPoint, T, inlierMask);
    if (isRegistered) {
        cout << "Register succeed " << id << '\n';
        // update frame
        frame->setRegistered(true);
        frame->setTcw(T);

        // update track
        for (int i = 0; i < inlierMask.size(); ++i) {
            if (!inlierMask[i]) continue;
            map->addObservation(mapPointId[i], id, kpIdx[i]);
        }

        // triangulate and update map
        vector<vector<Map::CorrData>> data;
        map->getMatchToNeighbor(frame, data);
        triangulate(data);

        return true;
    } else {
        cout << "Register failed " << id << '\n';
        return false;
    }
}

// Triangulate newly registered frame with its neighbors.
void Reconstruction::triangulate(const vector<vector<Map::CorrData>> &data) {
    for (auto &corrData:data) {   // for each point
        if (data.size() < 2) continue;

        Track track;
        vector<cv::Mat> Ps;
        vector<cv::Point2f> points2d;
        for (auto singleViewData:corrData) {    // for each view
            track.addElement(singleViewData.frameId, singleViewData.kptIdx);
            Ps.push_back(singleViewData.P);
            points2d.push_back(singleViewData.kpt);
        }

        cv::Point3f pt3d;
        triangulator->triangulateMultiviews(points2d, Ps, pt3d);

        // update map
        map->insertMapPoint(pt3d, track);
    }
}

}