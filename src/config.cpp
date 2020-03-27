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

namespace sfm{

std::shared_ptr<Config> Config::instance() {
    static std::shared_ptr<Config> config_ = nullptr;
    if (config_ == nullptr)
        config_ = std::shared_ptr<Config>(new Config);

    return config_;
}

void Config::setParameterFile(const std::string &filename) {
    file_ = new cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if (!file_->isOpened()) {
        cerr << "parameter file " << filename << " doesn't exist.\n";
        file_->release();
    }
}

Config::~Config() {
    if (file_->isOpened())
        file_->release();
    delete file_;
}

}