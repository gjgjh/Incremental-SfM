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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "sfm/common.h"
#include "sfm/track.h"

namespace sfm {

class MapPoint {
public:
    typedef shared_ptr<MapPoint> Ptr;

    MapPoint(long id, const cv::Vec3f &pos, const cv::Vec3b &color) : id(id), pos(pos), color(color) {}
    ~MapPoint() {};
    static MapPoint::Ptr createMapPoint(const cv::Vec3f &pos, const cv::Vec3b &color);

    // setter and getter
    void setTrack(const Track &track) { MapPoint::track = track; }
    Track &getTrack() { return track; }
    long getId() const { return id; }
    const cv::Vec3f &getPos() const { return pos; }
    void setPos(const cv::Vec3f &pos) { MapPoint::pos = pos; }
    const cv::Vec3b &getColor() const { return color; }
    void setColor(const cv::Vec3b &color) { MapPoint::color = color; }

private:
    long id;
    cv::Vec3f pos;                                        // position in world
    cv::Vec3b color;                                      // b, g, r
    Track track;                                          // frameID_keypointIdx
};

}

#endif // MAPPOINT_H