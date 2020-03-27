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

#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include "sfm/map.h"
#include "sfm//optimizer.h"
#include "sfm/initializer.h"
#include "sfm/triangulator.h"
#include "sfm/pnpSolver.h"

namespace sfm {

class Reconstruction {
public:
    Reconstruction();
    ~Reconstruction() {}

    void run();
    void writePLYBinary(const string &path);

private:
    bool tryInitialize(int maxTrial);

    bool registerNext(Frame::Ptr frame);
    void triangulate(const vector<vector<Map::CorrData>> &data);

private:
    Map::Ptr map;
    Optimizer::Ptr optimizer;
    Initializer::Ptr initializer;
    Triangulator::Ptr triangulator;
    PnPSolver::Ptr pnpSolver;

    int numRegistered = 0;
    int lastNumRegistered = 1;
    int localOptInterval;              // local BA optimization interval
    float globalOptRatio;              // when the model size increases by a certain ratio, then perform global BA
};

}

#endif // RECONSTRUCTION_H