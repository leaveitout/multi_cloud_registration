//
// Created by sean on 10/11/15.
//

#include "KeypointSorter.hpp"
#include "Logger.h"

bool KeypointSorter::areKeypointsValid(const vector<vector<cv::Point2d>>& keypoints) {
    if(keypoints.size() == NUM_SQUARES)
        return keypoints[0].size() == NUM_CORNERS && keypoints[1].size() == NUM_CORNERS;
}

bool KeypointSorter::sortKeyPointsLeft(vector<vector<cv::Point2d>>& keypoints) {
    if(!areKeypointsValid(keypoints))
        return false;

    sort(keypoints.begin(), keypoints.end(), compare_vertical_squares_inverse());

    for(auto& sq : keypoints) {
        sort(sq.begin(), sq.end(), compare_horizontal());
        sort(sq.begin(), sq.begin()+2, compare_vertical_inverse());
        sort(sq.begin()+2, sq.end(), compare_vertical_inverse());
    }
    return true;
}

bool KeypointSorter::sortKeyPointsCenter(vector<vector<cv::Point2d>>& keypoints) {
    if(!areKeypointsValid(keypoints))
        return false;

    sort(keypoints.begin(), keypoints.end(), compare_horizontal_squares_inverse());

    for(auto& sq : keypoints) {
        sort(sq.begin(), sq.end(), compare_vertical_inverse());
        sort(sq.begin(), sq.begin()+2, compare_horizontal_inverse());
        sort(sq.begin()+2, sq.end(), compare_horizontal_inverse());
    }
    return true;
}

bool KeypointSorter::sortKeyPointsRight(vector<vector<cv::Point2d>>& keypoints) {
    if(!areKeypointsValid(keypoints))
        return false;

    sort(keypoints.begin(), keypoints.end(), compare_vertical_squares());

    for(auto& sq : keypoints) {
        sort(sq.begin(), sq.end(), compare_horizontal_inverse());
        sort(sq.begin(), sq.begin()+2, compare_vertical());
        sort(sq.begin()+2, sq.end(), compare_vertical());
    }
    return true;
}

cv::Point2d KeypointSorter::getCentroid(const vector<cv::Point2d>& keypoints) {
    cv::Point2d centroid{0,0};
    for(auto& kp: keypoints)
        centroid += kp;
    int size = (int) keypoints.size();
    centroid = cv::Point2d(centroid.x/size, centroid.y/size);
    return move(centroid);
}
