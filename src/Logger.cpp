#include "Logger.h"
#include <mutex>
#include <memory>

namespace Logger {

    namespace {
        std::unique_ptr<std::mutex> io_mutex_;
    }

    std::once_flag flag;

    void log(Logger::Level level, const char *format, ...) {
        std::call_once(flag, [](){io_mutex_.reset(new std::mutex());});

        if (!pcl::console::isVerbosityLevelEnabled((pcl::console::VERBOSITY_LEVEL) level)) return;
        FILE *stream = (level == WARN || level == ERROR) ? stderr : stdout;
        switch (level) {
            case DEBUG:
                change_text_color(stream, TT_RESET, TT_GREEN);
                break;
            case WARN:
                change_text_color(stream, TT_BRIGHT, TT_YELLOW);
                break;
            case ERROR:
                change_text_color(stream, TT_BRIGHT, TT_RED);
                break;
            case ALWAYS:
            case INFO:
            case VERBOSE:
            default:
                break;
        }

        va_list ap;
        va_start (ap, format);
        {
            std::lock_guard<std::mutex> lock(*io_mutex_);
            vfprintf(stream, format, ap);
        }
        va_end (ap);

        reset_text_color(stream);
    }
}
