cmake_minimum_required(VERSION 3.3)

project(multi_cloud_registration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES src/main.cpp src/App.cpp include/App.h src/Logger.cpp
        include/Viewer.h include/ViewerNode.h include/QRDetector.h src/QRDetector.cpp
        src/SquareDetector.cpp include/SquareDetector.h src/KeypointSorter.cpp include/KeypointSorter.hpp include/CameraId.hpp include/Palette.hpp)
include_directories(${CMAKE_HOME_DIRECTORY}/include)

add_executable(multi_cloud_registration ${SOURCE_FILES})

target_link_libraries(multi_cloud_registration ${PCL_LIBRARIES})
target_link_libraries(multi_cloud_registration ${OpenCV_LIBS})
