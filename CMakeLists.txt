cmake_minimum_required(VERSION 2.8.3)
project(traditional_stereo_ros)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES Image.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES sensor_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

## Add OpenCV dependencies
find_package(OpenCV)

## Build "depth_listener" and "stereo_sender"
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Build the "stereo_sender" executable
add_executable(stereo_sender src/stereo_sender.cpp)
add_dependencies(stereo_sender ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(stereo_sender ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Build the "depth_listener" executable
add_executable(depth_listener src/depth_listener.cpp)
add_dependencies(depth_listener ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(depth_listener ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
