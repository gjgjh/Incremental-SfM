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

#include "sfm/matcher.h"
#include "sfm/config.h"

int main(int argc, char **argv) {
    using namespace sfm;

    if (argc != 2) {
        cerr << "Usage: ComputeMatches <ConfigFilePath>\n";
        return -1;
    }
    cout << "================================\n";
    cout << "Compute matches\n";
    cout << "================================\n";

    // load some parameters
    Config::instance()->setParameterFile(argv[1]);
    int match_type=Config::instance()->get<int>("match_type");
    float ratio_thresh = Config::instance()->get<float>("ratio_thresh");
    float distance_thresh = Config::instance()->get<float>("distance_thresh");
    int min_num_matches = Config::instance()->get<int>("min_num_matches");
    int cross_check = Config::instance()->get<int>("cross_check");

    shared_ptr<Matcher> matcher;
    if (match_type == 0) {
        int overlap = Config::instance()->get<int>("sequential.overlap");
        matcher = shared_ptr<Matcher>(new Sequential(overlap));
    } else if (match_type == 1) {
        matcher = shared_ptr<Matcher>(new Exhaustive);
    } else if (match_type == 2) {
        matcher = shared_ptr<Matcher>(new VocabTree);
    } else {
        cerr << "Match type should be 0, 1 or 2\n";
        return -1;
    }

    matcher->setRatioThresh(ratio_thresh).setDistanceThresh(distance_thresh).setMinNumMatches(
            min_num_matches).setCrossCheck(cross_check).setVisualize(false);

    matcher->matchAll();

    return 0;
}