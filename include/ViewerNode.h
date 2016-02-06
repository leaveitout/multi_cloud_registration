//
// Created by sean on 08/10/15.
//

#ifndef PCL_MULTI_CAMERA_REGISTRATION_VIEWERNODE_H
#define PCL_MULTI_CAMERA_REGISTRATION_VIEWERNODE_H


#include <mutex>
#include <memory>
#include <pcl/io/openni2_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "Timer.h"
#include "SquareDetector.h"
#include "Palette.hpp"


template <typename PointType>
class ViewerNode {
private:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef std::vector<std::vector<cv::Point2d>> Corners;

    pcl::io::OpenNI2Grabber::Ptr grabber_;

    std::string name_;

    boost::signals2::connection cloud_connection_;
    boost::signals2::connection image_connection_;

    std::mutex cloud_mutex_;
    std::mutex image_mutex_;
    std::mutex corners_mutex_;

    CloudConstPtr cloud_;
    CloudConstPtr corners_cloud_;

    boost::shared_ptr<pcl::io::openni2::Image> image_;
    unsigned char *rgb_data_;
    unsigned rgb_data_size_;
    std::shared_ptr<Corners> corners_;
    std::shared_ptr<Palette> palette_;

    std::unique_ptr<Timer> cloud_timer_, image_timer_;

    void cloudCallback(const CloudConstPtr& cloud) {
        cloud_timer_->time();
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        cloud_ = cloud;
    }

    void imageCallback(const boost::shared_ptr<pcl::io::Image> &image) {
        image_timer_->time();
        std::unique_lock<std::mutex> image_lock(image_mutex_);
        image_ = image;

        if (image->getEncoding() != pcl::io::openni2::Image::RGB) {
            if (rgb_data_size_ < image->getWidth() * image->getHeight()) {
                if (rgb_data_)
                    delete[] rgb_data_;
                rgb_data_size_ = image->getWidth() * image->getHeight();
                rgb_data_ = new unsigned char[rgb_data_size_ * 3];
            }
            image_->fillRGB(image_->getWidth(), image_->getHeight(), rgb_data_);
        }

        cv::Mat frame(image->getHeight(), image->getWidth(), CV_8UC3, (void *) image->getData());

        // TODO: Move this out of the data acquisition code
//        image_lock.unlock();
        std::lock_guard<std::mutex> corners_lock(corners_mutex_);
        auto point_indices = SquareDetector::getPointIndicesOfCorners(frame, name_, palette_);
        image_lock.unlock();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr correspondence_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // Extract the inliers
        // Create the filtering object
        // TODO: Check timing to make sure that the image and cloud are the same time
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        std::unique_lock<std::mutex> cloud_lock(cloud_mutex_);
        extract.setInputCloud (cloud_);
        extract.setIndices (point_indices);
        extract.setNegative (false);
        extract.filter (*correspondence_cloud);
        cloud_lock.unlock();
        std::cerr << "PointCloud representing the keypoints: " <<
                correspondence_cloud->width * correspondence_cloud->height << " data points." << std::endl;
//        for(auto i: point_indices->indices)
//            Logger::log(Logger::INFO, std::to_string(i));
//        corners_ = SquareDetector::detectSquareCorners(frame, name_);
//        SquareDetector::drawSquareCorners(frame, corners_, palette_);
    }

public:

    ViewerNode(pcl::io::OpenNI2Grabber::Ptr grabber, std::string name, std::shared_ptr<Palette> palette)
            : grabber_ (grabber)
            , name_ (name)
            , palette_ (palette)
            , rgb_data_ ()
            , rgb_data_size_ () {
        std::stringstream ss1;
        ss1 << "Camera " << name_ << " ";
        cloud_timer_.reset(new Timer(ss1.str().append("Cloud Timer")));
        image_timer_.reset(new Timer(ss1.str().append("Image Timer")));
    }

    ~ViewerNode() {
        cloud_connection_.disconnect ();
        image_connection_.disconnect ();

        if (rgb_data_)
            delete[] rgb_data_;
    }

    void setup() {
        boost::function<void (const CloudConstPtr&) > cloud_callback =
                boost::bind(&ViewerNode::cloudCallback, this, _1);
        cloud_connection_ = grabber_->registerCallback(cloud_callback);

        if (providesImageCallback()) {
            boost::function<void (const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb =
                    boost::bind (&ViewerNode::imageCallback, this, _1);
            image_connection_ = grabber_->registerCallback (image_cb);
        }
    }

    std::string getName() {
        return name_;
    }

    bool providesImageCallback() {
        typedef const boost::shared_ptr<pcl::io::openni2::Image>& NI2ImageConstPtr;
        return grabber_->providesCallback<void (NI2ImageConstPtr)>();
    }

    void getCloud(CloudConstPtr& cloud) {
        // TODO: Unique lock?
        if(cloud_mutex_.try_lock()) {
            cloud_.swap(cloud);
            cloud_mutex_.unlock();
        }
    }

    bool getImage(boost::shared_ptr<pcl::io::openni2::Image>& image) {
        if(providesImageCallback())
            if(image_mutex_.try_lock()) {
                image_.swap(image);
                image_mutex_.unlock();
                return true;
            }
        return false;
    }

    void getCornersCloud(CloudConstPtr& corners_cloud) {
        if(corners_mutex_.try_lock()) {
            corners_cloud_.swap(corners_cloud);
            corners_mutex_.unlock();
        }
    }
//    void getCorners(std::shared_ptr<Corners>& corners) {
//        if(corners_mutex_.try_lock()) {
//            corners_.swap(corners);
//            corners_mutex_.unlock();
//        }
//    }
};

#endif //PCL_MULTI_CAMERA_REGISTRATION_VIEWERNODE_H
