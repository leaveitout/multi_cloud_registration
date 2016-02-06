//
// Created by sean on 16/11/15.
//

#ifndef MULTI_CLOUD_REGISTRATION_THREADEDVIEWER_HPP
#define MULTI_CLOUD_REGISTRATION_THREADEDVIEWER_HPP

#include <memory>

#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <thread>

#include "ViewerNodeThreaded.hpp"
#include "CameraId.hpp"

typedef pcl::io::OpenNI2Grabber NI2Grabber;
typedef pcl::io::openni2::OpenNI2DeviceManager NI2DeviceManager;

template <typename PointType>
class ThreadedViewer {
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef boost::shared_ptr<pcl::io::openni2::Image> NI2ImagePtr;
    typedef typename pcl::visualization::ImageViewer ImageViewer;
    typedef typename pcl::visualization::PCLVisualizer CloudViewer;

private:
    std::unique_ptr<CloudViewer> cloud_viewer_;
    std::vector<std::shared_ptr<ImageViewer>> image_viewers_;
    std::vector<std::shared_ptr<ViewerNodeThreaded<PointType>>> nodes_;
    std::vector<NI2ImagePtr> images_;
    std::vector<CloudConstPtr> clouds_;
    std::mutex cloud_viewer_mutex_;
    std::mutex image_viewer_mutexes_[3];

    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*) {
        std::string special_key = event.getKeyCode() ? "" : "special ";
        if(event.keyDown())
            Logger::log(Logger::INFO, "The %skey %c was pressed.", special_key.c_str(), event.getKeyCode() );
        else
            Logger::log(Logger::INFO, "The %skey %c was released.", special_key.c_str(), event.getKeyCode() );
    }

    void mouseCallback(const pcl::visualization::MouseEvent& mouse_event, void*) {
        if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress &&
            mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton) {
            cout << "Left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
        }
    }

    void getAndProcessImage(size_t node_index) {
        while(!checkCloseCondition()) {
//            image_viewers_.at(node_index)->spinOnce(33, false);
            NI2ImagePtr image = nodes_.at(node_index)->getImage();

            /// TODO: All the processing here

            std::lock_guard<std::mutex> lock(image_viewer_mutexes_[node_index]);
            image.swap(images_.at(node_index));

//            std::lock_guard<std::mutex> lock(image_viewer_mutexes_[node_index]);
//            if (image->getEncoding() == pcl::io::openni2::Image::RGB)
//                image_viewers_.at(node_index)->addRGBImage((const unsigned char *) image->getData(),
//                                                           image->getWidth(), image->getHeight());
//            // TODO: Give access to rgb_data_ of the node
////            else
//            Logger::log(Logger::INFO, "Got image.\n");
////                image_viewers_.at(node_index)->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
//            image_viewers_.at(node_index)->spinOnce(33);
//            boost::this_thread::sleep(boost::posix_time::microseconds(33000));
        }
    }

    void getAndProcessCloud(size_t node_index) {
        while(!checkCloseCondition()){
//            cloud_viewer_->spin();
//            cloud_viewer_->spinOnce(33, false);
            CloudConstPtr cloud = nodes_.at(node_index)->getCloud();
            std::string cloud_name = nodes_.at(node_index)->getCameraId();

            std::lock_guard<std::mutex> lock(cloud_viewer_mutex_);
            if (!cloud_viewer_->updatePointCloud (cloud, cloud_name)) {
                cloud_viewer_->addPointCloud (cloud, cloud_name);
                if((nodes_.at(node_index)->getCameraId() == CameraId::center)) {
                    cloud_viewer_->resetCameraViewpoint (cloud_name);
                    cloud_viewer_->setCameraPosition (
                            0,0,0,		// Position
                            0,0,1,		// Viewpoint
                            0,-1,0);	// Up
                }
            }

//            std::this_thread::sleep_for(std::chrono::milliseconds(30));

        }
    }

    bool checkCloseCondition() {
        if(cloud_viewer_->wasStopped())
            return true;

        // NB: No elements in the vector if no nodes provide image callback
        for(auto& iv: image_viewers_)
            if(iv->wasStopped())
                return true;

        return false;
    }

public:
    ThreadedViewer(NI2Grabber::Ptr left, NI2Grabber::Ptr center, NI2Grabber::Ptr right) {
        std::shared_ptr<ViewerNodeThreaded<PointType>> node_left;
        node_left.reset(new ViewerNodeThreaded<PointType>(left, CameraId::left));
        nodes_.push_back(node_left);
        std::shared_ptr<ViewerNodeThreaded<PointType>> node_center;
        node_center.reset(new ViewerNodeThreaded<PointType>(center, CameraId::center));
        nodes_.push_back(node_center);
        std::shared_ptr<ViewerNodeThreaded<PointType>> node_right;
        node_right.reset(new ViewerNodeThreaded<PointType>(right, CameraId::right));
        nodes_.push_back(node_right);
    }

    void run() {
        // TODO: Set up 6 threads, one for each image and cloud
        cloud_viewer_.reset(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
        cloud_viewer_->registerMouseCallback(&ThreadedViewer::mouseCallback, *this);
        cloud_viewer_->registerKeyboardCallback(&ThreadedViewer::keyboardCallback, *this);
        // TODO: Check this number for Asus Xtion
        cloud_viewer_->setCameraFieldOfView(1.02259994f);
        cloud_viewer_->setPosition(0, 1000);
        // TODO: This in terms of screen height and screen width
        cloud_viewer_->setSize(640, 480);

        int image_viewer_count = 0;
        for(auto& node : nodes_) {
            node->setup();
//            cloud_viewer_mutexes_.push_back(std::mutex())

            if(node->providesImageCallback()) {
                auto image_viewer = std::make_shared<ImageViewer>(node->getCameraId() + "Image");
                image_viewer->registerMouseCallback(&ThreadedViewer::mouseCallback, *this);
                image_viewer->registerKeyboardCallback(&ThreadedViewer::keyboardCallback, *this);
                // TODO: Remove literals
                image_viewer->setPosition(image_viewer_count*640, 0);
                image_viewer->setSize(640, 480);
                image_viewers_.push_back(image_viewer);
                images_.push_back(NI2ImagePtr());
//                image_viewer_mutexes_.push_back(std::mutex());
                image_viewer_count++;
            }
        }

        // Start the threads
        std::vector<std::thread> threads;
//        size_t  image_node_index = 0;
        for(size_t node_index = 0; node_index < nodes_.size(); ++node_index) {
            threads.push_back(std::thread(&ThreadedViewer::getAndProcessCloud, this, node_index));
            if(nodes_.at(node_index)->providesImageCallback()) {
                threads.push_back(std::thread(&ThreadedViewer::getAndProcessImage, this, node_index));
//                image_node_index++;
            }
        }

        while(!checkCloseCondition()) {
            {
//                std::lock_guard<std::mutex> lock(image_viewer_mutexes_[0]);
                if(image_viewer_mutexes_[0].try_lock()) {

                    Logger::log(Logger::INFO, "The image viewer was spun.\n");
//                cloud_viewer_->spinOnce(100);
//                image_viewers_.at(0)->spinOnce(50);
                    if (images_.at(0)->getEncoding() == pcl::io::openni2::Image::RGB)
                        image_viewers_.at(0)->addRGBImage((const unsigned char *) images_.at(0)->getData(),
                                                          images_.at(0)->getWidth(), images_.at(0)->getHeight());
                    image_viewers_.at(0)->spin();
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }

        for(auto& thread : threads)
            thread.join();
    }

};

#endif //MULTI_CLOUD_REGISTRATION_THREADEDVIEWER_HPP
