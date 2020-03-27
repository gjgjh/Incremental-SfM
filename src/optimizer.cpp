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

#include "sfm/optimizer.h"
#include "sfm/utils.h"

namespace sfm {

Optimizer::Optimizer() {
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 *blockSolver = new g2o::BlockSolver_6_3(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

    float fx = Camera::instance()->getFx();
    float cx = Camera::instance()->getCx();
    float cy = Camera::instance()->getCy();
    camera = new g2o::CameraParameters(fx, Eigen::Vector2d(cx, cy), 0);
    camera->setId(0);

    robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");
}

void Optimizer::globalBA(Map::Ptr map) {
    const auto &mapPoints = map->getAllMapPoints();
    const auto &frames = map->getAllFrames();

    // only get all registered frames
    unordered_map<long, Frame::Ptr> registeredFrames;
    for (auto &frame:frames) {
        if (!frame.second->isRegistered()) continue;
        registeredFrames[frame.first] = frame.second;
    }

    bundleAdjustment(registeredFrames, mapPoints);
    recoverOptimizedData(registeredFrames, mapPoints);
}

void Optimizer::localBA(Map::Ptr map, Frame::Ptr currentFrame) {
    // get covisible neighbor frames (not all connected neighbors) from track
    unordered_map<long, Frame::Ptr> frames;
    frames.insert({currentFrame->getId(), currentFrame});

    bool exceedMaxNum = false;
    const vector<MapPoint::Ptr> &currentMapPoints = currentFrame->getMapPoints();
    for (auto &point:currentMapPoints) {
        if (point.get() == nullptr) continue;
        if (exceedMaxNum) break;

        const auto &trackElements = point->getTrack().getElements();
        for (auto element:trackElements) {
            long neighborFrameId = element.first;
            frames.insert({neighborFrameId, map->getAllFrames().at(neighborFrameId)});

            if (frames.size() > maxLocalBAFramesNum) {
                exceedMaxNum = true;
                break;
            }
        }
    }

    // get local map points seen in these local frames
    unordered_map<long, MapPoint::Ptr> localMapPoints;
    for (auto it = frames.begin(); it != frames.end(); ++it) {
        const auto &mapPoints = it->second->getMapPoints();
        for (auto &point:mapPoints) {
            if (point.get() == nullptr) continue;
            localMapPoints.insert({point->getId(), point});
        }
    }

    bundleAdjustment(frames, localMapPoints);
    recoverOptimizedData(frames, localMapPoints);
}

// clear and reset nodes and edges
void Optimizer::reset() {
    optimizer = new g2o::SparseOptimizer;
    optimizer->setAlgorithm(solver);
    optimizer->setVerbose(true);
    optimizer->addParameter(camera);
}

void Optimizer::bundleAdjustment(const unordered_map<long, Frame::Ptr> &frames,
                                 const unordered_map<long, MapPoint::Ptr> &mappoints) {
    // reset all BA data
    reset();

    // set frame vertices
    maxFrameId = -1;
    for (auto &id_frame:frames) {
        long frameId = id_frame.first;
        Frame::Ptr frame = id_frame.second;
        maxFrameId = max(maxFrameId, frameId);

        auto *v = new g2o::VertexSE3Expmap;
        v->setId(frameId);
        v->setEstimate(Utils::toSE3Quat(frame->getTcw()));
        optimizer->addVertex(v);
    }

    // set map point vertices
    for (auto &id_points:mappoints) {
        long pointId = id_points.first;
        MapPoint::Ptr point = id_points.second;

        auto *v = new g2o::VertexSBAPointXYZ();
        long vertexId = maxFrameId + pointId + 1;   // in order not to conflict with the frame ID
        v->setId(vertexId);
        v->setMarginalized(true);
        v->setEstimate(Utils::toVector3d(point->getPos()));
        optimizer->addVertex(v);

        const auto &tracks = point->getTrack();

        // set edges
        for (auto &track:tracks.getElements()) {
            long frameId = track.first;
            int kpId = track.second;
            if (frames.find(frameId) == frames.end()) continue;

            auto *edge = new g2o::EdgeProjectXYZ2UV();

            edge->setVertex(0, optimizer->vertex(vertexId));
            edge->setVertex(1, optimizer->vertex(frameId));

            Eigen::Matrix<double, 2, 1> obs;
            auto kp = frames.at(frameId)->getKp()[kpId].pt;
            obs << kp.x, kp.y;
            edge->setMeasurement(obs);
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(robustKernel);
            edge->setParameterId(0, 0);
            optimizer->addEdge(edge);
        }
    }

    // optimize!
    cout << "Start optimization\n";
    optimizer->initializeOptimization();
    optimizer->optimize(iteration);
}

// Recover pose and map points from optimized data.
void Optimizer::recoverOptimizedData(const unordered_map<long, Frame::Ptr> &frames,
                                     const unordered_map<long, MapPoint::Ptr> &mappoints) {
    // frames
    for (auto &id_frame:frames) {
        long frameId = id_frame.first;
        Frame::Ptr frame = id_frame.second;

        auto *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer->vertex(frameId));
        g2o::SE3Quat SE3quat = v->estimate();
        frame->setTcw(Utils::toIsometry3d(SE3quat)); // fixme: may be inverse?
    }

    // map points
    for (auto &id_points:mappoints) {
        long pointId = id_points.first;
        MapPoint::Ptr point = id_points.second;

        long vertexId = maxFrameId + pointId + 1;
        auto *v = dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer->vertex(vertexId));
        point->setPos(Utils::toCvMat(v->estimate()));
        v->setId(vertexId);
    }
}

void Optimizer::saveOptResult(string filepath) const {
    optimizer->save(filepath.c_str());
}

}
