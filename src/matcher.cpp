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

#include "sfm/matcher.h"
#include "sfm/database.h"
#include "sfm/config.h"

namespace sfm {

Matcher::Matcher() {
    string dbPath=Config::instance()->get<string>("database_path");
    database.Open(dbPath);
}

// Match two image, return whether the matching is successful.
bool Matcher::matchTwo(long id1, long id2, vector<cv::DMatch> &matches) {
    cout << "Match between " + to_string(id1) + ' ' + to_string(id2) + '\n';
    cv::Mat desp1 = database.ReadDescriptors(id1);
    cv::Mat desp2 = database.ReadDescriptors(id2);

    vector<cv::DMatch> originalMatches;
    matchTwoImpl(desp1, desp2, originalMatches);

    if (originalMatches.size() < minNumMatches) {
        cout << "Good matches number is too small\n";
        return false;
    }

    filterByMatchDistance(originalMatches, originalMatches, distanceThresh);

    vector<cv::KeyPoint> kp1 = database.ReadKeyPoints(id1);
    vector<cv::KeyPoint> kp2 = database.ReadKeyPoints(id2);

    filterByGeoConstraint(kp1, kp2, originalMatches, originalMatches);

    matches = originalMatches;

    string imageDir=Config::instance()->get<string>("image_dir");
    string imgPath1 = imageDir + '/' + database.ReadImageById(id1).name;
    string imgPath2 = imageDir + '/' + database.ReadImageById(id2).name;
    cv::Mat img1 = cv::imread(imgPath1);
    cv::Mat img2 = cv::imread(imgPath2);
    if (visualize) visualizing(img1, img2, kp1, kp2, matches);

    return true;
}

void Matcher::matchTwoImpl(const cv::Mat &desp1, const cv::Mat &desp2, vector<cv::DMatch> &matches) {
    if (crossCheck == 0) {
        knnMatch(desp1, desp2, matches, ratioThresh);
    } else if (crossCheck == 1) {
        vector<cv::DMatch> matches12;
        vector<cv::DMatch> matches21;

        knnMatch(desp1, desp2, matches12, ratioThresh);
        knnMatch(desp2, desp1, matches21, ratioThresh);

        filterByCrossCheck(matches12, matches21, matches);
    } else {
        cerr << "Cross check should be 0 or 1\n";
    }
}

// flann matching and ratio test
void Matcher::knnMatch(const cv::Mat &desp1, const cv::Mat &desp2, vector<cv::DMatch> &matches, float ratioThresh) {
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    vector<vector<cv::DMatch>> knn_matches;
    matcher->knnMatch(desp1, desp2, knn_matches, 2);

    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratioThresh * knn_matches[i][1].distance) {
            matches.push_back(knn_matches[i][0]);
        }
    }
}

// you need me, I do need you
void Matcher::filterByCrossCheck(const vector<cv::DMatch> &matches12, const vector<cv::DMatch> &matches21,
                                 vector<cv::DMatch> &matches) {
    std::unordered_map<int, int> matchMap;
    for (size_t i = 0; i < matches21.size(); ++i) {
        int query_idx = matches21[i].queryIdx;
        int train_idx = matches21[i].trainIdx;
        matchMap[query_idx] = train_idx;
    }

    int good_matches = 0;
    for (size_t i = 0; i < matches12.size(); ++i) {

        int query_idx = matches12[i].queryIdx;
        int train_idx = matches12[i].trainIdx;

        if (matchMap[train_idx] == query_idx) {
            good_matches += 1;
            matches.push_back(matches12[i]);
        }
    }
}

// Filter by match distance.
// Note that matches and goodMatches can be the same object.
void Matcher::filterByMatchDistance(const vector<cv::DMatch> &matches, vector<cv::DMatch> &goodMatches,
                                    float maxDistance) {
    vector<cv::DMatch> filteredMatches;
    for (size_t i = 0; i < matches.size(); ++i) {
        if (matches[i].distance > maxDistance)
            continue;
        filteredMatches.push_back(matches[i]);
    }
    goodMatches = std::move(filteredMatches);
}

// Filter by geometric constraint (Fundamental matrix) and RANSAC.
// Note that matches and goodMatches can be the same object.
void Matcher::filterByGeoConstraint(const vector<cv::KeyPoint> &kp1, const vector<cv::KeyPoint> &kp2,
                                    const vector<cv::DMatch> &matches, vector<cv::DMatch> &filteredMatches) {
    vector<cv::Point2f> pts1;
    vector<cv::Point2f> pts2;
    for (cv::DMatch match: matches) {
        int queryIdx = match.queryIdx;
        int trainIdx = match.trainIdx;
        pts1.push_back(kp1[queryIdx].pt);
        pts2.push_back(kp2[trainIdx].pt);
    }

    // note that the definition of inliers here is different from the definition in the cv::solvePnPRansac function
    cv::Mat inliers;
    cv::findFundamentalMat(pts1, pts2, inliers, cv::FM_RANSAC, 3, 0.99);

    vector<cv::DMatch> inlierMatches;
    for (int i = 0; i < inliers.rows; i++) {
        if (inliers.at<uchar>(i, 0) == 0) continue;
        inlierMatches.push_back(matches[i]);
    }
    filteredMatches = inlierMatches;
}

void Matcher::visualizing(const cv::Mat &img1, const cv::Mat &img2, const vector<cv::KeyPoint> &kp1,
                          const vector<cv::KeyPoint> &kp2, const vector<cv::DMatch> &matches) {
    cv::Mat img_matches;
    cv::drawMatches(img1, kp1, img2, kp2, matches, img_matches, cv::Scalar(0, 0, 255),
                    cv::Scalar(0, 255, 0), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imshow("match result", img_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void Sequential::matchAll() {
    if (overlap < 1) {
        cerr << "Overlap should be at least 1\n";
        return;
    }

    vector<Database::Image> images = database.ReadAllImages();
    for (int i = 0; i < images.size(); ++i) {
        int imageId1 = images[i].id;
        for (int j = 1; j <= overlap; ++j) {
            if (i + j >= images.size()) break;
            int imageId2 = images[i + j].id;

            // skip if match already exists
            if (database.ExistMatches(imageId1, imageId2)) continue;

            vector<cv::DMatch> matches;
            bool isSuccessful = matchTwo(imageId1, imageId2, matches);

            database.BeginTransaction();
            if (isSuccessful) database.WriteMatches(imageId1, imageId2, matches);
            database.EndTransaction();
        }
    }
}

void Exhaustive::matchAll() {

}

void VocabTree::matchAll() {

}

}
