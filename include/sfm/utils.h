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

#ifndef UTILS_H
#define UTILS_H

#include "sfm/common.h"

namespace sfm {

// some helper functions
class Utils {
public:
    Utils() {}

    virtual ~Utils() {}

    static vector<string> loadImages(string image_path);
    static string pathTofilename(const string &filepath);
    static string unionPath(const string &directory, const string &filename);
    static Isometry3d matToIsometry(const cv::Mat &R, const cv::Mat &t);
    static Vector3d toVector3d(const cv::Vec3d &vec);
    static cv::Vec3d toCvMat(const Vector3d &vec);
    static g2o::SE3Quat toSE3Quat(const Isometry3d &T);
    static Isometry3d toIsometry3d(const g2o::SE3Quat& SE3Quat);
};

}

#endif // UTILS_H