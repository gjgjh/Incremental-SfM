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

#ifndef TRIANGULATOR_H
#define TRIANGULATOR_H

#include "sfm/common.h"
#include "sfm/frame.h"

namespace sfm{

class Triangulator{
public:
    typedef shared_ptr<Triangulator> Ptr;

    void triangulate(const vector<vector<cv::Point2f>>& points2d,const vector<cv::Mat>& Ps,vector<cv::Point3f>& points3d);
    void triangulateTwoFrames(Frame::Ptr frame1, Frame::Ptr frame2, vector<cv::Point3f>& points3d);
    void triangulateMultiviews(const vector<cv::Point2f>& points2d, const vector<cv::Mat>& Ps, cv::Point3f& point3d);
};

}

#endif