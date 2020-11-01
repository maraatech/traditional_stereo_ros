cmake_minimum_required(VERSION 2.8.3)
project(traditional_stereo_ros)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES Message.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build "depth_listener" and "stereo_sender"
include_directories(include ${catkin_INCLUDE_DIRS})

## Build the "stereo_sender" executable
add_executable(stereo_sender src/stereo_sender.cpp)
target_link_libraries(stereo_sender ${catkin_LIBRARIES})
add_dependencies(stereo_sender traditional_stereo_ros_generate_messages_cpp)

## Build the "depth_listener" executable
add_executable(depth_listener src/depth_listener.cpp)
target_link_libraries(depth_listener ${catkin_LIBRARIES})
add_dependencies(depth_listener traditional_stereo_ros_generate_messages_cpp)
