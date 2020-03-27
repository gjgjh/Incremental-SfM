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

#ifndef MATCHER_H
#define MATCHER_H

#include "sfm/common.h"
#include "sfm/database.h"

namespace sfm {

// strategy class
class Matcher {
public:
    Matcher();
    virtual ~Matcher() { database.Close(); };

    static void knnMatch(const cv::Mat &desp1, const cv::Mat &desp2, vector<cv::DMatch> &matches, float ratioThresh);
    static void filterByCrossCheck(const vector<cv::DMatch> &matches12, const vector<cv::DMatch> &matches21,
                                   vector<cv::DMatch> &matches);
    static void filterByMatchDistance(const vector<cv::DMatch> &matches, vector<cv::DMatch> &filteredMatches,
                                      float maxDistance);
    static void filterByGeoConstraint(const vector<cv::KeyPoint> &kp1, const vector<cv::KeyPoint> &kp2,
                                      const vector<cv::DMatch> &matches, vector<cv::DMatch> &inlierMatches);
    static void visualizing(const cv::Mat &img1, const cv::Mat &img2, const vector<cv::KeyPoint> &kp1,
                            const vector<cv::KeyPoint> &kp2, const vector<cv::DMatch> &matches);

    virtual void matchAll() = 0;

    Matcher &setRatioThresh(float ratioThresh) {
        Matcher::ratioThresh = ratioThresh;
        return *this;
    }

    Matcher &setDistanceThresh(float distanceThresh) {
        Matcher::distanceThresh = distanceThresh;
        return *this;
    }

    Matcher &setMinNumMatches(int minNumMatches) {
        Matcher::minNumMatches = minNumMatches;
        return *this;
    }

    Matcher &setCrossCheck(int crossCheck) {
        Matcher::crossCheck = crossCheck;
        return *this;
    }

    Matcher &setVisualize(bool visualize) {
        Matcher::visualize = visualize;
        return *this;
    }

protected:
    bool matchTwo(long id1, long id2, vector<cv::DMatch> &matches);

    Database database;

private:
    void matchTwoImpl(const cv::Mat &desp1, const cv::Mat &desp2, vector<cv::DMatch> &matches);

    float ratioThresh = 0.7;
    float distanceThresh = 0.7;
    int minNumMatches = 15;
    int crossCheck = 1;
    bool visualize = false;
};

// Sequential matching
class Sequential : public Matcher {
public:
    Sequential(int overlap) : overlap(overlap) {}
    virtual ~Sequential() {}

    void matchAll() override;

private:
    int overlap = 3;
};

// Exhaustive matching
class Exhaustive : public Matcher {
public:
    Exhaustive() : Matcher() {}
    virtual ~Exhaustive() {};

    void matchAll() override;
};

// VocabTree matching
class VocabTree : public Matcher {
public:
    VocabTree() : Matcher() {}
    virtual ~VocabTree() {};

    void matchAll() override;
};

}

#endif // MATCHER_H
