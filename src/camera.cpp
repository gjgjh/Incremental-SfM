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

#include "sfm/camera.h"
#include "sfm/config.h"

namespace sfm {

Camera::Camera() {
    fx = Config::instance()->get<float>("camera.fx");
    fy = Config::instance()->get<float>("camera.fy");
    cx = Config::instance()->get<float>("camera.cx");
    cy = Config::instance()->get<float>("camera.cy");

    k1 = Config::instance()->get<float>("camera.k1");
    k2 = Config::instance()->get<float>("camera.k2");
    p1 = Config::instance()->get<float>("camera.p1");
    p2 = Config::instance()->get<float>("camera.p2");
}

Camera::Ptr Camera::instance() {
    static std::shared_ptr<Camera> camera = nullptr;
    if (camera == nullptr)
        camera = std::shared_ptr<Camera>(new Camera);

    return camera;
}

cv::Mat Camera::K() const {
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx,
            0, fy, cy,
            0, 0, 1);
    return K;
}

cv::Mat Camera::distCoef() const {
    cv::Mat distCoef = (cv::Mat_<double>(4, 1) << k1, k2, p1, p2);
    return distCoef;
}

}