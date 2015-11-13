//
// Created by sean on 06/11/15.
//

#include "SquareDetector.h"

#include <forward_list>

#include <pcl/PointIndices.h>

#include "Logger.h"
#include "CameraId.hpp"


//pcl::PointIndices::Ptr SquareDetector::getPointIndicesOfCoArners(shared_ptr<vector<vector<Point2d>>> corners, Mat image) {
//
//}

shared_ptr<vector<vector<Point2d>>> SquareDetector::detectSquareCorners(Mat image, string camera_id) {
    if(image.empty()){
        Logger::log(Logger::ERROR, "No image to detect");
        return nullptr;
    }

    Mat gray;
    cvtColor(image, gray, CV_BGR2GRAY);
    auto corners = detectCorners(gray);
//    for(auto& corner : corners)
//        circle(image, corner, CIRCLE_RADIUS, Scalar(255,0,0), CIRCLE_THICKNESS);
    auto squares = detectSquares(gray);
//    drawContours(image, squares, -1, Scalar(0,0,255));
    auto square_corners(resolveCornersToSquares(corners, squares));
    bool valid = false;

    if(camera_id.compare(CameraId::left) == 0)
        valid = KeypointSorter::sortKeyPointsLeft(*square_corners);
    else if(camera_id.compare(CameraId::center) == 0)
        valid = KeypointSorter::sortKeyPointsCenter(*square_corners);
    else if(camera_id.compare(CameraId::right) == 0)
        valid = KeypointSorter::sortKeyPointsRight(*square_corners);

    return valid ? square_corners : nullptr;
}

// Returns a set of contours representing the found squares in the image
vector<vector<Point>> SquareDetector::detectSquares(cv::Mat image) {

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cv::threshold(image, image, 0, CHANNEL_DEPTH, CV_THRESH_BINARY | CV_THRESH_OTSU);
    findContours(image, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    vector<vector<Point>> square_contours;
    for(auto& c : contours) {
        // TODO: Add as parameters
        if(contourArea(c) > MIN_CONTOUR_AREA && contourArea(c) < MAX_CONTOUR_AREA) {
            vector<Point> approx;
            cv::approxPolyDP(cv::Mat(c), approx, arcLength(cv::Mat(c), true) * APPROX_POLY_LENGTH_RATIO, true);
            if(approx.size() == 4 && isContourConvex(Mat(approx))){
                double max_cosine = 0;

                for( int j = 2; j < 5; j++ ) {
                    double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                    max_cosine = MAX(max_cosine, cosine);
                }

                if( max_cosine < MAX_COSINE_LIMIT ) {
                    square_contours.push_back(approx);
                }
            }
        }
    }
    return move(square_contours);
}

vector<Point2d> SquareDetector::detectCorners(Mat image) {
    vector<cv::Point2d> corners;
    goodFeaturesToTrack( image,
                         corners,
                         MAX_CORNERS,
                         QUALITY_LEVEL,
                         MIN_DISTANCE,
                         noArray(),
                         BLOCK_SIZE,
                         USE_HARRIS,
                         DEFAULT_K);

    return std::move(corners);
}

shared_ptr<vector<vector<Point2d>>> SquareDetector::resolveCornersToSquares(const vector<Point2d> &corners,
                                                                const vector<vector<Point>> &squares) {
    forward_list<cv::Point2d> corners_list(corners.begin(), corners.end());
    shared_ptr<vector<vector<Point2d>>> squares_corners ( new vector<vector<Point2d>>);

    for(auto& sq: squares) {
        vector<Point2d> single_square_corners;
        for (auto& pt: sq) {
            for (auto& c : corners_list){
                if (dist(pt, c) < MAX_DIST_CORNER_TO_CONTOUR_PT) {
                    single_square_corners.push_back(c);
                    corners_list.remove(c);
                    break;
                }
            }
        }
        if(single_square_corners.size() == 4) {
            squares_corners->push_back(single_square_corners);
        }
    }

    return squares_corners;
}


void SquareDetector::drawSquareCorners(Mat image,
                                       shared_ptr<vector<vector<Point2d>>> squareCorners,
                                       shared_ptr<Palette> palette) {
    if(!image.data || !squareCorners || !palette)
        return;

    if(squareCorners->size() != 2)
        return;

    int color_index = 0;
    for(auto& sq : *squareCorners)
        for(auto& corner : sq) {
            circle(image, corner, CIRCLE_RADIUS, palette->getColorAt(color_index), CIRCLE_THICKNESS);
            color_index++;
            putText(image, to_string(color_index), corner, FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,255,0));
        }
}

double SquareDetector::angle( cv::Point2d pt1, cv::Point2d pt2, cv::Point2d pt0 ) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

double SquareDetector::dist(Point2d pt0, Point2d pt1) {
    Point2d d = pt1 - pt0;
    return cv::sqrt(d.x*d.x + d.y*d.y);
}

Scalar SquareDetector::getRandomColor() {
    random_device rd;
    uniform_int_distribution<int> dist(0, CHANNEL_DEPTH);
    Scalar color = Scalar(dist(rd), dist(rd), dist(rd));
    return move(color);
}

