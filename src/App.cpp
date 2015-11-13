//
// Created by sean on 03/11/15.
//

#include <Logger.h>
#include "App.h"

App::App(int argc, char **argv) {
    // Parse arguments
    // Check that cameras are connected
}

int App::exec() {
    auto device_manager = NI2DeviceManager::getInstance();

    std::vector<NI2Grabber::Ptr> grabbers;

    auto numDevices = device_manager->getNumOfConnectedDevices();

    for(auto i=1; i<=numDevices; ++i) {
        std::stringstream ss;
        ss << '#' << i;
        NI2Grabber::Ptr grabber(new NI2Grabber(ss.str()));
        grabbers.push_back(grabber);
    }

    for(auto& g: grabbers) {
        Logger::log(Logger::INFO, g->getDevice()->getUri() + "\n");
        g->start();
    }

    viewer_.reset(new Viewer<pcl::PointXYZRGBA>(grabbers.at(1), grabbers.at(0), grabbers.at(2)));
    viewer_->run();

    for (auto g : grabbers) {
        g->stop();
    }

    return 0;
}
