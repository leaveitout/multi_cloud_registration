//
// Created by sean on 03/11/15.
//

#ifndef MULTI_CLOUD_REGISTRATION_APP_H
#define MULTI_CLOUD_REGISTRATION_APP_H

#include <memory>

#include <pcl/io/openni2_grabber.h>
#include "Viewer.h"

typedef pcl::io::OpenNI2Grabber NI2Grabber;
typedef pcl::io::openni2::OpenNI2DeviceManager NI2DeviceManager;

class App {
private:
    std::unique_ptr<Viewer<pcl::PointXYZRGBA>> viewer_;
//    std::unique_ptr<MultiOpenNI2Viewer<pcl::PointXYZRGBA> > viewer;
    void parseArguments();
    void printHelp();



public:
    App(int argc, char** argv);

    int exec();
};


#endif //MULTI_CLOUD_REGISTRATION_APP_H
