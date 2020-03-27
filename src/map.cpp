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

#include "sfm/config.h"
#include "sfm/map.h"

namespace sfm {

// load matches and keypoints from database into map
void Map::load() {
    string databasePath = Config::instance()->get<string>("database_path");
    Database database;
    database.Open(databasePath);

    loadMatches(database);
    loadKeypoints(database);

    database.Close();
}

// Insert a map point.
void Map::insertMapPoint(const cv::Vec3f &pos, const Track &track) {
    // average the colors of multiple points on a track
    int sumB = 0, sumG = 0, sumR = 0;
    for (auto &trackItem:track.getElements()) {
        long frameId = trackItem.first;
        int kpIdx = trackItem.second;
        Frame::Ptr frame = frames[frameId];
        cv::Vec3b color = frame->getColor()[kpIdx];

        sumB += color(0);
        sumG += color(1);
        sumR += color(2);
    }

    cv::Vec3b color(static_cast<uchar>(sumB / track.length()),
                    static_cast<uchar>(sumG / track.length()),
                    static_cast<uchar>(sumR / track.length()));

    // insert map point
    MapPoint::Ptr mapPoint = MapPoint::createMapPoint(pos, color);
    mapPoint->setTrack(track);
    mapPoints[mapPoint->getId()] = mapPoint;

    // set frame associated information
    for (auto &trackItem:track.getElements()) {
        long frameId = trackItem.first;
        int kpIdx = trackItem.second;
        Frame::Ptr frame = frames[frameId];
        frame->setMapPoint(kpIdx, mapPoint);
    }
}

// Add a new frame observation to the map point.
void Map::addObservation(long mapPointId, long frameId, int kpIdx) {
    assert(hasMapPoint(mapPointId));

    Frame::Ptr newFrame = frames[frameId];
    MapPoint::Ptr mapPoint = mapPoints[mapPointId];

    // update color
    cv::Vec3b oldColor = mapPoint->getColor();
    cv::Vec3b color = newFrame->getColor()[kpIdx];
    int length = mapPoint->getTrack().length();

    int oldB = oldColor(0), oldG = oldColor(1), oldR = oldColor(2);
    int newB = color(0), newG = color(1), newR = color(2);
    cv::Vec3b newColor(
            static_cast<unsigned char>((oldB * length + newB) / (length + 1)),
            static_cast<unsigned char>((oldG * length + newG) / (length + 1)),
            static_cast<unsigned char>((oldR * length + newR) / (length + 1)));
    mapPoint->setColor(newColor);

    // update track
    mapPoint->getTrack().addElement(frameId, kpIdx);
    newFrame->setMapPoint(kpIdx, mapPoint);
}

// Check map point exists.
bool Map::hasMapPoint(long id) {
    return mapPoints.find(id) != mapPoints.end();
}

// Check whether the given observation is part of a two-view track, i.e.
// it only has one correspondence and that correspondence has the given
// observation as its only correspondence.
bool Map::isTwoViewObservation(Frame::Ptr frame, int kpId) {
    auto id_matches = frame->getMatches();
    auto mathces = id_matches.at(kpId);
    if (mathces.size() != 1) return false;

    long otherFrameId = mathces.begin()->first;
    int otherKptId = mathces.begin()->second;
    Frame::Ptr otherFrame = frames[otherFrameId];
    return otherFrame->getMatches().at(otherKptId).size() == 1;
}

// load frames and matches from database
void Map::loadMatches(const Database &database) {
    int min_num_matches = Config::instance()->get<int>("min_num_matches");
    vector<pair<int, vector<cv::DMatch>>> matches = database.ReadAllMatches();
    for (auto &match: matches) {
        int id1, id2, pairId = match.first;
        vector<cv::DMatch> matchData = match.second;
        Database::PairIdToImagePair(pairId, &id1, &id2);

        if (matchData.size() < min_num_matches) continue;

        Frame::Ptr frame;
        if (frames.find(id1) == frames.end()) {
            frame = Frame::createFrame(id1);
            frames[id1] = frame;
        }
        if (frames.find(id2) == frames.end()) {
            frame = Frame::createFrame(id2);
            frames[id2] = frame;
        }

        addMatch(frames[id1], frames[id2], matchData);
    }
}

// load and undistort keypoints from database
void Map::loadKeypoints(const Database &database) {
    for (auto &frame:frames) {
        int id = frame.first;
        Frame::Ptr pFrame = frame.second;
        vector<cv::KeyPoint> kpts = database.ReadKeyPoints(id);
        vector<cv::Vec3b> colors = database.ReadKeyPointsColor(id);
        vector<MapPoint::Ptr> mappoints;
        mappoints.resize(kpts.size());

        pFrame->setKp(kpts);
        pFrame->setColor(colors);
        pFrame->setMapPoints(mappoints);

        pFrame->undistortKeypoint(camera->K(), camera->distCoef());
    }
}

// Find seed images for incremental reconstruction.
// The returned list is ordered such that most suitable images are in the front.
vector<Frame::Ptr> Map::findFirstInitialImage() const {
    vector<Frame::Ptr> candidates;
    for (auto &frame:frames) {
        Frame::Ptr pFrame = frame.second;
        if (pFrame->getNumRegister() > 0) continue; // skip Frames that failed to initialize

        candidates.push_back(pFrame);
    }

    sort(candidates.begin(), candidates.end(), [](Frame::Ptr frame1, Frame::Ptr frame2) {
        return frame1->getMatches().size() > frame2->getMatches().size();
    });

    return candidates;
}

// For a given first seed image, find other images that are connected to the first image.
// Suitable second images have a large number of correspondences to the first image.
// The returned list is ordered such that most suitable images are in the front.
vector<Frame::Ptr> Map::findSecondInitialImage(Frame::Ptr firstFrame) {

    // find images that are connected to the first seed image and have
    // not been registered before in other reconstructions
    unordered_map<long, int> id_numMatches;
    for (auto &match:firstFrame->getMatches()) {
        auto id_KpIdx = match.second;
        for (auto &it:id_KpIdx) {
            long id = it.first;
            if (frames[id]->getNumRegister() > 0) continue; // skip Frames that failed to initialize

            id_numMatches[id] += 1;
        }
    }

    vector<pair<long, int>> id_numMatches_vec;
    for (auto &id_numMatch:id_numMatches) {
        long id = id_numMatch.first;
        int numMatch = id_numMatch.second;
        id_numMatches_vec.push_back({id, numMatch});
    }

    // more correspondences are preferred when sorting
    sort(id_numMatches_vec.begin(), id_numMatches_vec.end(), [](pair<long, int> elem1, pair<long, int> elem2) {
        return elem1.second > elem2.second;
    });

    // extract Frames in sorted order
    vector<Frame::Ptr> candidates;
    for (auto &id_numMatch:id_numMatches_vec) {
        long id = id_numMatch.first;
        candidates.push_back(frames[id]);
    }

    return move(candidates);
}

// Get the images to be registered next time.
// Find images that are not yet registered and neighbors are registered.
// The returned list is ordered such that most suitable images are in the front.
vector<Frame::Ptr> Map::findNextRegisterImage() {
    vector<pair<long, int>> id_scores;
    for (auto &frame:frames) {
        Frame::Ptr pFrame = frame.second;
        long id = pFrame->getId();

        // skip if already registered
        if (pFrame->isRegistered()) continue;

        vector<long> neighborsId = pFrame->getNeighborsId();
        vector<long> registeredNeighborsId;
        for (auto neighborId:neighborsId) {
            if (frames[neighborId]->isRegistered()) registeredNeighborsId.push_back(neighborId);
        }

        // skip if neighbors are all not registered
        if (registeredNeighborsId.empty()) continue;

        id_scores.push_back({id, registeredNeighborsId.size()});
    }

    sort(id_scores.begin(), id_scores.end(), [](const pair<long, int> &id_scores1, const pair<long, int> &id_scores2) {
        return id_scores1.second > id_scores2.second;
    });

    vector<Frame::Ptr> candidates;
    for (auto &id_score:id_scores) {
        long id = id_score.first;
        candidates.push_back(frames[id]);
    }
    return move(candidates);
}

// Get matched keypoints between frame and map.
void Map::getMatchToMap(Frame::Ptr frame, vector<cv::Vec2f> &keypoint, vector<cv::Vec3f> &mapPoint, vector<int> &kpIdx,
                        vector<long> &mapPointId) {
    keypoint.clear();
    mapPoint.clear();
    kpIdx.clear();
    mapPointId.clear();
    auto matches = frame->getMatches();

    // search all neighbors
    for (auto &id_match:matches) {
        auto &match = id_match.second;
        for (auto &frameId_kptId:match) {
            long frameId = frameId_kptId.first;
            int kptId = frameId_kptId.second;

            // find the map points that are already in the map
            if (frames[frameId]->hasMapPoint(kptId)) {
                kpIdx.push_back(id_match.first);
                keypoint.push_back(frame->getKp()[id_match.first].pt);

                auto mappoint = frames[frameId]->getMapPoints()[kptId];
                mapPointId.push_back(mappoint->getId());
                mapPoint.push_back(mappoint->getPos());
                break;
            }
        }
    }
}

// Get matched keypoints between all neighbors, mainly used to triangulate.
// Inner vector is multiview observation, outside vector is keypoints.
void Map::getMatchToNeighbor(Frame::Ptr frame, vector<vector<CorrData>> &data) {
    data.clear();

    // search all neighbors
    auto matches = frame->getMatches();
    for (auto &id_match:matches) {  // for each keypoint
        long kptId = id_match.first;
        auto &match = id_match.second;

        // skip keypoint if already have associated mappoint
        if (frame->hasMapPoint(kptId)) continue;

        // skip if it is a two-view track (not very useful in reconstruction)
        if (isTwoViewObservation(frame, kptId)) continue;

        vector<CorrData> dataTemp;
        for (auto &frameId_kptId:match) {   // for each neighbor frame
            long otherFrameId = frameId_kptId.first;
            int otherKptId = frameId_kptId.second;

            // skip if neighbor frame is not registered
            bool isRegistered = frames[otherFrameId]->isRegistered();
            if (!isRegistered) continue;

            // skip neighbor keypoint if it already has associated mappoint
            if (frames[otherFrameId]->hasMapPoint(otherKptId)) continue;

            cv::Mat P;
            cv::eigen2cv(frames[otherFrameId]->getTcw().matrix(), P);
            cv::Mat K = Camera::instance()->K();
            P = K * P.rowRange(0, 3); // P=K[R|t]

            CorrData corrData;
            corrData.frameId = otherFrameId;
            corrData.kptIdx = otherKptId;
            corrData.P = P;
            corrData.kpt = frames[otherFrameId]->getKp()[otherKptId].pt;
            dataTemp.push_back(corrData);
        }

        // add current frame itself for multiview triangulation
        if (!dataTemp.empty()) {
            cv::Mat P;
            cv::eigen2cv(frame->getTcw().matrix(), P);
            cv::Mat K = Camera::instance()->K();
            P = K * P.rowRange(0, 3); // P=K[R|t]

            CorrData corrData;
            corrData.frameId = frame->getId();
            corrData.kptIdx = kptId;
            corrData.P = P;
            corrData.kpt = frame->getKp()[kptId].pt;
            dataTemp.push_back(corrData);

            data.push_back(dataTemp);
        }
    }
}

void Map::writePLYBinary(const string &path) {
    std::ofstream file;
    // trunc: delete the file if it already exists
    file.open(path.c_str(), std::ios::trunc);

    file << "ply" << std::endl;
    file << "format binary_little_endian 1.0" << std::endl;
    file << "element vertex " << mapPoints.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "end_header" << std::endl;

    for (auto &pts3d:mapPoints) {
        MapPoint::Ptr point = pts3d.second;
        cv::Vec3f pos = point->getPos();
        cv::Vec3b color = point->getColor();
        swap(color(0),color(2));    // opencv store color in B, G, R order

        file.write((char *) pos.val, sizeof(float) * 3);
        file.write((char *) color.val, sizeof(uchar) * 3);
    }

    file.close();
}

}