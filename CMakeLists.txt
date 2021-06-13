cmake_minimum_required(VERSION 2.8.3)
project(traditional_stereo_ros)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        message_filters
        message_generation
        image_transport
        sensor_msgs
        cares_msgs)

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES Image.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES sensor_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS cv_bridge message_filters message_runtime sensor_msgs)

## Add OpenCV dependencies
find_package(OpenCV)

## Build "depth_listener" and "stereo_sender"
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Build the "Start" executable
add_executable(traditional_stereo
                        src/traditional_stereo_node.cpp
                        src/parameters.h
                        src/StereoPipeline.h
                        src/StereoPipeline.cpp
                        src/TraditionalStereo/SGBM.cpp
                        src/TraditionalStereo/Math3D.cpp
                        src/TraditionalStereo/LoadUtils.cpp
                        src/TraditionalStereo/OpticsUtils.cpp
                        src/TraditionalStereo/StereoFrameUtils.cpp
                        src/TraditionalStereo/GeneralUtils.cpp
                        src/TraditionalStereo/DisplayUtils.cpp
                        src/TraditionalStereo/CloudUtils.cpp
                        src/TraditionalStereo/PointSaver.cpp)
add_dependencies(traditional_stereo ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(traditional_stereo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
