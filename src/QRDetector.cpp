//
// Created by sean on 02/10/15.
//

#include "QRDetector.h"

std::vector<Point2d> QRDetector::detectQrCodes(Mat image) {
    RNG rng(12345);

    if(image.empty()){
        std::cerr << "Error: no image to detect.";
        return std::vector<Point2d>();
    }

    gray.create(image.size(), CV_MAKETYPE(image.depth(), 1));
    binary.create(image.size(), CV_MAKETYPE(image.depth(), 1));

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    cvtColor(image, gray, CV_BGR2GRAY);
//    GaussianBlur(gray, gray, Size(3,3), 1, 1);
//    Canny(gray, edges, 100, 200, 3);
//    findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    threshold(gray, binary, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    vector<int> deep_contours = findDeepContours(contours, hierarchy, 3);

    vector<Point2d> results;

    for(int i=0; i<deep_contours.size(); ++i) {
        if(!isContourConvex(contours[deep_contours[i]]) && contourArea(contours[deep_contours[i]]) > MIN_AREA) {
            int parent_index = hierarchy[deep_contours[i]][3];
            if (parent_index != -1) {
                int grandparent_index = hierarchy[parent_index][3];
                double child_area = contourArea(contours[deep_contours[i]]);
                double parent_area_ratio = contourArea(contours[parent_index]) / child_area;
                double grandparent_area_ratio = contourArea(contours[grandparent_index]) / child_area;
                if(PARENT_AREA_RATIO_MIN < parent_area_ratio && parent_area_ratio < PARENT_AREA_RATIO_MAX)
                    if(GRANDPARENT_AREA_RATIO_MIN < grandparent_area_ratio &&
                            grandparent_area_ratio < GRANDPARENT_AREA_RATIO_MAX) {
                        Point2d center = calcContourCOG(contours[grandparent_index]);
                        results.push_back(center);

                        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
                        drawContours(image, contours, grandparent_index, color, 2, LINE_8, hierarchy, 2);
                        image.at<Vec3b>(Point2i(center)) = Vec3b((uchar)color[0], (uchar)color[1], (uchar)color[2]);
                    }
            }
        }
    }

    // Sort the landmarks into top-left, top-right, bottom-left, bottom-right
    if(sortLandmarks(results)) {
        return results;
    }
    else
        return std::vector<Point2d>();
}

int QRDetector::countParentContours(int contour_index, const vector<Vec4i> &hierarchy) {
    int depth = 1;
    int parent_index = hierarchy[contour_index][3];
    if(parent_index == -1)
        return depth;
    else
        return depth + countParentContours(parent_index, hierarchy);
}

vector<int> QRDetector::findDeepContours(const vector<vector<Point> > &contours,
                                         const vector<Vec4i> &hierarchy, int min_depth) {
    vector<int> deep_contours;
    for(int i=0; i< contours.size(); ++i)
        if(countParentContours(i, hierarchy) > min_depth)
            deep_contours.push_back(i);
    return deep_contours;
}

double QRDetector::approxPolyArea(vector<Point> &contour, double epsilon) {
    vector<Point> poly;
    approxPolyDP(Mat(contour), poly, epsilon, true);
    return contourArea(poly, false);
}

bool QRDetector::sortLandmarks(vector<Point2d> &points) {
    if(points.size() == 4) {
        struct compare_vertical {
            inline bool operator() (const Point2d& pt1, const Point2d& pt2) {
                return pt1.y < pt2.y;
            }
        };

        struct compare_horizontal {
            inline bool operator() (const Point2d& pt1, const Point2d& pt2) {
                return pt1.x < pt2.x;
            }
        };

        sort(points.begin(), points.end(), compare_vertical());
        sort(points.begin(), points.begin()+2, compare_horizontal());
        sort(points.begin()+2, points.end(), compare_horizontal());
    }
    else
        return false;
}

Point2d QRDetector::calcContourCOG(vector<Point> &contour) {
    Moments mu = moments(contour, false);
    return Point2d(mu.m10/mu.m00, mu.m01/mu.m00);
}
