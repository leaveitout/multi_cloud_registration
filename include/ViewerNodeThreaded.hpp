//
// Created by sean on 16/11/15.
//

#ifndef MULTI_CLOUD_REGISTRATION_VIEWERNODETHREADED_HPP
#define MULTI_CLOUD_REGISTRATION_VIEWERNODETHREADED_HPP

#include <mutex>
#include <memory>
#include <pcl/io/openni2_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <condition_variable>

#include "Timer.h"
#include "CameraId.hpp"


template <typename PointType>
class ViewerNodeThreaded {
private:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef typename boost::shared_ptr<pcl::io::openni2::Image> ImagePtr;

    pcl::io::OpenNI2Grabber::Ptr grabber_;

    std::string camera_id_;

    boost::signals2::connection cloud_connection_;
    boost::signals2::connection image_connection_;

    std::mutex cloud_mutex_;
    std::mutex image_mutex_;

    std::condition_variable cloud_cv_;
    std::condition_variable image_cv_;

    bool cloud_available_;
    bool image_available_;

    CloudConstPtr cloud_;
    boost::shared_ptr<pcl::io::openni2::Image> image_;
    unsigned char *rgb_data_;
    unsigned rgb_data_size_;

    std::unique_ptr<Timer> cloud_timer_, image_timer_;

    void cloudCallback(const CloudConstPtr& cloud) {
        cloud_timer_->time();
        {
            std::unique_lock<std::mutex> lock(cloud_mutex_);
            cloud_ = cloud;
            cloud_available_ = true;
        }
        cloud_cv_.notify_all();
    }

    void imageCallback(const boost::shared_ptr<pcl::io::Image> &image) {
        image_timer_->time();
        {
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
            image_available_ = true;
        }
        image_cv_.notify_all();
    }

public:

    ViewerNodeThreaded(pcl::io::OpenNI2Grabber::Ptr grabber, std::string camera_id) :
            grabber_ (grabber),
            camera_id_ (camera_id),
            rgb_data_ (),
            rgb_data_size_ (),
            image_available_ (false),
            cloud_available_ (false) {
        std::stringstream ss1;
        ss1 << "Camera " << camera_id_ << " ";
        cloud_timer_.reset(new Timer(ss1.str().append("Cloud Timer")));
        image_timer_.reset(new Timer(ss1.str().append("Image Timer")));
    }

    ~ViewerNodeThreaded() {
        cloud_connection_.disconnect ();
        image_connection_.disconnect ();

        if (rgb_data_)
            delete[] rgb_data_;
    }

    void setup() {
        boost::function<void (const CloudConstPtr&) > cloud_callback =
                boost::bind(&ViewerNodeThreaded::cloudCallback, this, _1);
        cloud_connection_ = grabber_->registerCallback(cloud_callback);

        if (providesImageCallback()) {
            boost::function<void (const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb =
                    boost::bind (&ViewerNodeThreaded::imageCallback, this, _1);
            image_connection_ = grabber_->registerCallback (image_cb);
        }
    }

    std::string getCameraId() const {
        return camera_id_;
    }

    bool providesImageCallback() const {
        typedef const boost::shared_ptr<pcl::io::openni2::Image>& NI2ImageConstPtr;
        return grabber_->providesCallback<void (NI2ImageConstPtr)>();
    }

    const CloudConstPtr getCloud() {
        std::unique_lock<std::mutex> lock(cloud_mutex_);
        cloud_cv_.wait(lock, [&](){ return cloud_available_; });
//        Logger::log(Logger::INFO, "Cloud available for node %s\n", camera_id_.c_str());
        cloud_available_ = false;
        CloudConstPtr cloud;
        cloud_.swap(cloud);
        return cloud;
//        return cloud_;
    }

    const ImagePtr getImage() {
        std::unique_lock<std::mutex> lock(image_mutex_);
        image_cv_.wait(lock, [&](){ return image_available_; });
        image_available_ = false;
        ImagePtr image;
        image_.swap(image);
        return image;
    }

};

#endif //MULTI_CLOUD_REGISTRATION_VIEWERNODETHREADED_HPP
