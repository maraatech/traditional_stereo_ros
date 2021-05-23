# Traditional Stereo ROS
Traditional Stereo matching (not machine learning based) and depth map calculation = NOTE that images might need to be scaled down to enable "real-time" performance.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
1) ROS Noetic

2) Pull master version of cares_msgs
    a) cd ~/catkin_ws/src
    b) git clone https://github.com/UoA-CARES/cares_msgs.git

3) Compile both libraries
    a) cd ~/catkin_ws
    b) catkin_make
```

### Installing

A step by step series of examples that tell you how to get a development env running

Clone the package into the catkin directory you are using, presumed here to be "~/catkin_ws"

```
cd ~/catkin_ws/src
git clone https://github.com/maraatech/traditional_stereo_ros.git
```

Build the package with catkin_make in the source directory

```
cd ~/catkin_src/
catkin_make
```

## Running the tests

Tests to be added

## Applications

### Stereo Depth
Subscribes to stereo image pairs with corresponding stereo camera information and constructs a 3D point cloud.
A scale factor can be applied to scale the size of images being processed to improve real-time performance.

#### Subscribed Topics
Left and right image pairs being published by a stereo pair with stereo camera information.
Topic names are all default names (left/right), they can be changed via setting parameters in the launch file and refer to topic names.

* sensor_msgs/Image
    * left/image_raw
    * right/image_raw
* cares_mgs/StereoCameraInfo
    * left_right/stereo_info

#### Published Topics
Publishes the point cloud and depth map generated by the stereo pairs + stereo information.

* sensor_msgs/PointCloud2
    * left_right/point_cloud
* sensor_msgs/Image
    * left_right/depth_image

##### live_stereo_calibration.launch
Run using launch file as below.

```
roslaunch traditional_stereo_ros stereo_depth.launch
```

```xml
<?xml version="1.0"?>
<launch>
  <arg name="left_image"  default="left/image_raw"/>
  <arg name="right_image" default="right/image_raw"/>
  <arg name="stereo_info" default="left_right/stereo_info"/>
  <arg name="point_cloud" default="left_right/point_cloud"/>
  <arg name="depth_image" default="left_right/depth_image"/>
  <arg name="scale"       default="1.0"/>

  <node name="traditional_stereo" pkg="traditional_stereo_ros" type="traditional_stereo" output="screen">
    <param name ="left_image"   value="$(arg left_image)"/>
    <param name ="right_image"  value="$(arg right_image)"/>
    <param name ="stereo_info"  value="$(arg stereo_info)"/>
    <param name ="point_cloud"  value="$(arg point_cloud)"/>
    <param name ="depth_image"  value="$(arg depth_image)"/>
    <param name ="scale"        value="$(arg scale)"/>
  </node>
</launch>
```
