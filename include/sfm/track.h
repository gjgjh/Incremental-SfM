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

#ifndef TRACK_H
#define TRACK_H

#include "sfm/common.h"

namespace sfm{

class Track{
public:
    void addElement(long frameId, int kpIdx);
    int length()const{ return elements.size();}

    const unordered_map<long, int> &getElements() const { return elements; }

private:
    unordered_map<long, int> elements;      // frameId and keypoint index
};

}

#endif