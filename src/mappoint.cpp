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

#include "sfm/mappoint.h"

namespace sfm {

MapPoint::Ptr MapPoint::createMapPoint(const cv::Vec3f &pos, const cv::Vec3b &color) {
    static long factory_id = 0;
    return MapPoint::Ptr(new MapPoint(factory_id++, pos, color));
}

}