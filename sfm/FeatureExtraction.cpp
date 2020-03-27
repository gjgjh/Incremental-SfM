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

#include "sfm/utils.h"
#include "sfm/frame.h"
#include "sfm/database.h"
#include "sfm/config.h"

int main(int argc, char **argv) {
    using namespace sfm;

    if (argc != 2) {
        cerr << "Usage: FeatureExtraction <ConfigFilePath>\n";
        return -1;
    }
    cout << "================================\n";
    cout << "Feature extraction\n";
    cout << "================================\n";

    // load image and some parameters
    Config::instance()->setParameterFile(argv[1]);
    string dbPath=Config::instance()->get<string>("database_path");
    string imageDir=Config::instance()->get<string>("image_dir");
    vector<string> imagePath = Utils::loadImages(imageDir);
    int maxNumFeatures=Config::instance()->get<int>("max_num_features");
    int maxImageSize=Config::instance()->get<int>("max_image_size");

    Database database;
    database.Open(dbPath);

    // detect keypoints and save to DB
    for (auto path: imagePath) {
        cv::Mat image = cv::imread(path);
        string filename = Utils::pathTofilename(path);
        cout << "Extracting " << filename << '\n';

        Frame::Ptr frame = Frame::createFrame(filename, image);
        frame->setMaxNumFeatures(maxNumFeatures);
        frame->setMaxImageSize(maxImageSize);
        frame->detectKeypoint(Frame::KeyPointType::SIFT);

        database.save(frame.get());
    }

    database.Close();

    return 0;
}
