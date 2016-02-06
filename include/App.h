//
// Created by sean on 03/11/15.
//

#ifndef MULTI_CLOUD_REGISTRATION_APP_H
#define MULTI_CLOUD_REGISTRATION_APP_H

#include <memory>

#include <pcl/io/openni2_grabber.h>
#include "ThreadedViewer.hpp"

typedef pcl::io::OpenNI2Grabber NI2Grabber;
typedef pcl::io::openni2::OpenNI2DeviceManager NI2DeviceManager;

class App {
private:
    void parseArguments();
    void printHelp();

public:
    App(int argc, char** argv);
    int exec();
};


#endif //MULTI_CLOUD_REGISTRATION_APP_H
