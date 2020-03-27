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

#ifndef CAMERA_H
#define CAMERA_H

#include "sfm/common.h"

namespace sfm {

// Pinhole camera model
class Camera {
public:
    typedef std::shared_ptr<Camera> Ptr;

    Camera();
    static Camera::Ptr instance();

    // getter
    cv::Mat K() const;
    cv::Mat distCoef() const;

    float getCx() const { return cx; }
    float getCy() const { return cy; }
    float getFx() const { return fx; }
    float getFy() const { return fy; }

private:
    float cx, cy, fx, fy;       // camera intrinsics
    float k1, k2, p1, p2;       // distortion
};

}

#endif // CAMERA_H