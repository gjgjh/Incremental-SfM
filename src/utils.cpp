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

namespace sfm {

// return all valid image paths
vector<string> Utils::loadImages(string image_path) {
    vector<cv::String> files;
    cv::glob(image_path, files);

    vector<string> imagePath;
    for (size_t i = 0; i < files.size(); ++i) {
        cv::Mat img = cv::imread(files[i]);
        if (img.empty()) {
            cerr << files[i] << " is invalid!" << endl;
            continue;
        } else {
            imagePath.push_back(files[i]);
        }
    }
    return imagePath;
}

// file path to file name
string Utils::pathTofilename(const string &filepath) {
    int i = filepath.size() - 1;
    while (i >= 0 && filepath[i] != '/')
        i--;
    return filepath.substr(i + 1, filepath.size());
}

string Utils::unionPath(const string &directory, const string &filename) {
    if (directory[directory.size() - 1] == '/') {
        return directory + filename;
    } else {
        return directory + "/" + filename;
    }
}

Isometry3d Utils::matToIsometry(const cv::Mat &R, const cv::Mat &t) {
    Matrix3d rotationMatrix;
    cv::cv2eigen(R, rotationMatrix);
    Vector3d tvec;
    cv::cv2eigen(t, tvec);

    Isometry3d T = Isometry3d::Identity();
    AngleAxisd rvec(rotationMatrix);
    T.rotate(rvec);
    T.pretranslate(tvec);

    return move(T);
}

Vector3d Utils::toVector3d(const cv::Vec3d &vec) {
    return Eigen::Vector3d(vec(0), vec(1), vec(2));
}

cv::Vec3d Utils::toCvMat(const Vector3d &vec) {
    return cv::Vec3d(vec.x(), vec.y(), vec.z());
}

g2o::SE3Quat Utils::toSE3Quat(const Isometry3d &T) {
    Matrix3d R = T.rotation();
    Vector3d t = T.translation();
    return g2o::SE3Quat(R, t);
}

Isometry3d Utils::toIsometry3d(const g2o::SE3Quat &SE3Quat) {
    Isometry3d T = Isometry3d::Identity();
    T.rotate(SE3Quat.rotation());
    T.pretranslate(SE3Quat.translation());
    return T;
}

}
