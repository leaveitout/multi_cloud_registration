//
// Created by sean on 02/10/15.
//

#ifndef PCL_QR_SEGMENTATION_QRDETECTOR_H
#define PCL_QR_SEGMENTATION_QRDETECTOR_H

#include <limits>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class QRDetector {

public:
    std::vector<Point2d> detectQrCodes(Mat image);
private:
    vector<int> findDeepContours(const vector<vector<Point> > &contours,
                                 const vector<Vec4i> &hierarchy, int min_depth);
    int countParentContours(int current_contour, const vector<Vec4i> &hierarchy);

    const double PARENT_AREA_RATIO_MIN = 2.1;
    const double PARENT_AREA_RATIO_MAX = 2.8;
    const double GRANDPARENT_AREA_RATIO_MIN = 4.8;
    const double GRANDPARENT_AREA_RATIO_MAX = 5.6;
    const int MIN_AREA = 100;

    Mat gray, binary;

    double approxPolyArea(vector<Point> &contour, double epsilon);

    Point2d calcContourCOG(vector<Point> &contour);

    bool sortLandmarks(vector<Point2d> &points);
};




#endif //PCL_QR_SEGMENTATION_QRDETECTOR_H
