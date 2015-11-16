//
// Created by sean on 03/11/15.
//

#ifndef MULTI_CLOUD_REGISTRATION_VIEWER_H
#define MULTI_CLOUD_REGISTRATION_VIEWER_H


#include <mutex>
#include <memory>

#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include "ViewerNode.h"
#include "QRDetector.h"
#include "CameraId.hpp"

typedef pcl::io::OpenNI2Grabber NI2Grabber;
typedef pcl::io::openni2::OpenNI2DeviceManager NI2DeviceManager;

template <typename PointType>
class Viewer {
private:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef typename pcl::visualization::ImageViewer ImageViewer;
    typedef typename pcl::visualization::PCLVisualizer CloudViewer;
    typedef std::vector<std::vector<cv::Point2d>> Corners;

    enum Camera {
        LEFT = 0,
        CENTER,
        RIGHT
    };

    bool cloud_viewer_init_;
    std::vector<bool> image_viewer_init_;

    std::unique_ptr<CloudViewer> cloud_viewer_;
    std::vector<std::shared_ptr<ImageViewer>> image_viewers_;

    std::vector<std::shared_ptr<ViewerNode<PointType>>> nodes_;
    Cloud left_keypoints;
    Cloud center_keypoints;
    Cloud right_keypoints;


    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*) {
        // TODO: Use Logger
        if (event.getKeyCode ())
            cout << "The key \'" << event.getKeyCode () << "\' (" << event.getKeyCode () << ") was";
        else
            cout << "The special key \'" << event.getKeySym () << "\' was";
        if (event.keyDown ())
            cout << " pressed." << endl;
        else
            cout << " released." << endl;
    }


    void mouseCallback(const pcl::visualization::MouseEvent& mouse_event, void*) {
        if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress &&
            mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton) {
            cout << "Left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
        }
    }

    void updateCloudViewer() {
        cloud_viewer_->spinOnce();

        updateCloudNode(LEFT);
        updateCloudNode(CENTER);
        updateCloudNode(RIGHT);
    }

    void updateCloudNode(Camera camera) {
        cloud_viewer_->spinOnce ();

        CloudConstPtr cloud;

        nodes_.at(camera)->getCloud(cloud);

        if(cloud) {
            if (!cloud_viewer_init_) {
                cloud_viewer_->setPosition (0, 0);
                cloud_viewer_->setSize (cloud->width, cloud->height);
                int *size =cloud_viewer_->getRenderWindow()->GetScreenSize();
                cout << "Size " << size[0] << ", " << size[1] << endl;
                cloud_viewer_init_ = true;
            }

            std::shared_ptr<Corners> corners;
//            nodes_.at(camera)->getCorners(corners);
            pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
//            indices->
            std::string cloud_name = nodes_.at(camera)->getName();
            if (!cloud_viewer_->updatePointCloud (cloud, cloud_name)) {
                cloud_viewer_->addPointCloud (cloud, cloud_name);
                if(camera == CENTER) {
                    cloud_viewer_->resetCameraViewpoint (cloud_name);
                    cloud_viewer_->setCameraPosition (
                            0,0,0,		// Position
                            0,0,1,		// Viewpoint
                            0,-1,0);	// Up
                }
            }
        }
    }


    void updateImageViewer(size_t node_index) {
        boost::shared_ptr<pcl::io::openni2::Image> image;
        // See if we can get an image
        nodes_.at(node_index)->getImage(image);

        if (image) {
            if (!image_viewer_init_.at(node_index)) {
                int count_init = 0;
                for(auto ii: image_viewer_init_)
                    if(ii)
                        count_init++;
                // TODO: Assumes all the images are of the same width
                image_viewers_.at(node_index)->setPosition(image->getWidth()*count_init, 0);
                image_viewers_.at(node_index)->setSize (image->getWidth(), image->getHeight());
                image_viewer_init_.at(node_index) = !image_viewer_init_.at(node_index);
            }

            if (image->getEncoding () == pcl::io::openni2::Image::RGB)
                image_viewers_.at(node_index)->addRGBImage ( (const unsigned char*)image->getData (),
                                                             image->getWidth (), image->getHeight ());
            // TODO: Give access to rgb_data_ of the node
//            else
//                image_viewers_.at(node_index)->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
            image_viewers_.at(node_index)->spinOnce ();
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

    Viewer(NI2Grabber::Ptr left, NI2Grabber::Ptr center, NI2Grabber::Ptr right)
            : cloud_viewer_init_ (false) {
        std::shared_ptr<Palette> palette(new Palette(8));

        std::shared_ptr<ViewerNode<PointType>> node_left;
        node_left.reset(new ViewerNode<PointType>(left, CameraId::left, palette));
        nodes_.push_back(node_left);
        std::shared_ptr<ViewerNode<PointType>> node_center;
        node_center.reset(new ViewerNode<PointType>(center, CameraId::center, palette));
        nodes_.push_back(node_center);
        std::shared_ptr<ViewerNode<PointType>> node_right;
        node_right.reset(new ViewerNode<PointType>(right, CameraId::right, palette));
        nodes_.push_back(node_right);
    }

    void run() {
        cloud_viewer_.reset(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
        cloud_viewer_->registerMouseCallback(&Viewer::mouseCallback, *this);
        cloud_viewer_->registerKeyboardCallback(&Viewer::keyboardCallback, *this);
        // TODO: Check this number for Asus Xtion
        cloud_viewer_->setCameraFieldOfView(1.02259994f);

        for(size_t i=0; i<nodes_.size(); ++i) {
            nodes_.at(i)->setup();

            if(nodes_.at(i)->providesImageCallback()) {
                std::shared_ptr<ImageViewer> viewer(new ImageViewer(nodes_.at(i)->getName() + "Image"));
                viewer->registerMouseCallback(&Viewer::mouseCallback, *this);
                viewer->registerKeyboardCallback(&Viewer::keyboardCallback, *this);
                image_viewers_.push_back(viewer);
                image_viewer_init_.push_back(false);
            }
        }

        while(!checkCloseCondition()) {
            for(size_t i=0; i<nodes_.size(); ++i) {
                updateCloudViewer();
                updateImageViewer(i);
            }
        }
    }
};

#endif //MULTI_CLOUD_REGISTRATION_VIEWER_H
