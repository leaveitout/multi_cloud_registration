#ifndef PCL_MULTI_OPENNI2_RECORDER_LOGGER_H
#define PCL_MULTI_OPENNI2_RECORDER_LOGGER_H

#include <string>
#include <pcl/console/print.h>

using namespace pcl::console;

namespace Logger {

    enum Level
    {
        ALWAYS = L_ALWAYS,
        ERROR = L_ERROR,
        WARN = L_WARN,
        INFO = L_INFO,
        DEBUG = L_DEBUG,
        VERBOSE = L_VERBOSE
    };

    void log(Logger::Level level, const char *format, ...);

    inline void log(Logger::Level level, const std::string& str) {
        log(level, str.c_str());
    }
}

#endif
