//
// Created by sean on 10/11/15.
//

#ifndef MULTI_CLOUD_REGISTRATION_FEATUREMATCHER_HPP
#define MULTI_CLOUD_REGISTRATION_FEATUREMATCHER_HPP

#include <opencv2/opencv.hpp>

using namespace std;

class KeypointSorter {
    friend class SquareDetector;

private:
    // Sorts keypoints in image space for different cameras
    static bool sortKeyPointsLeft(vector<vector<cv::Point2d>> &keypoints);

    static bool sortKeyPointsCenter(vector<vector<cv::Point2d>> &keypoints);

    static bool sortKeyPointsRight(vector<vector<cv::Point2d>> &keypoints);

    static cv::Point2d getCentroid(const vector<cv::Point2d> &keypoints);

    static bool areKeypointsValid(const vector<vector<cv::Point2d>> &keypoints);

    static constexpr int NUM_SQUARES = 2;

    static constexpr int NUM_CORNERS = 4;

    struct compare_vertical {
        inline bool operator()(const cv::Point2d &pt1, const cv::Point2d &pt2) {
            return pt1.y < pt2.y;
        }
    };

    struct compare_vertical_inverse {
        inline bool operator()(const cv::Point2d &pt1, const cv::Point2d &pt2) {
            return pt1.y > pt2.y;
        }
    };

    struct compare_horizontal {
        inline bool operator()(const cv::Point2d &pt1, const cv::Point2d &pt2) {
            return pt1.x < pt2.x;
        }
    };

    struct compare_horizontal_inverse {
        inline bool operator()(const cv::Point2d &pt1, const cv::Point2d &pt2) {
            return pt1.x > pt2.x;
        }
    };

    struct compare_horizontal_squares {
        inline bool operator()(const vector<cv::Point2d> &square1, const vector<cv::Point2d> &square2) {
            return KeypointSorter::getCentroid(square1).x < KeypointSorter::getCentroid(square2).x;
        }
    };

    struct compare_horizontal_squares_inverse {
        inline bool operator()(const vector<cv::Point2d> &square1, const vector<cv::Point2d> &square2) {
            return KeypointSorter::getCentroid(square1).x > KeypointSorter::getCentroid(square2).x;
        }
    };

    struct compare_vertical_squares {
        inline bool operator()(const vector<cv::Point2d> &square1, const vector<cv::Point2d> &square2) {
            return KeypointSorter::getCentroid(square1).y < KeypointSorter::getCentroid(square2).y;
        }
    };

    struct compare_vertical_squares_inverse {
        inline bool operator()(const vector<cv::Point2d> &square1, const vector<cv::Point2d> &square2) {
            return KeypointSorter::getCentroid(square1).y > KeypointSorter::getCentroid(square2).y;
        }
    };
};

#endif //MULTI_CLOUD_REGISTRATION_FEATUREMATCHER_HPP
