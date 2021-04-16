//--------------------------------------------------
// Implementation logic for the stereo pipeline
//
// @author: Wild Boar
//--------------------------------------------------

#include "StereoPipeline.h"  
using namespace Amantis;

//--------------------------------------------------
// Constructor and Terminator logic
//--------------------------------------------------

/**
 * Main constructor
 */
StereoPipeline::StereoPipeline(std::string point_cloud_node, std::string depth_node, double scale){
  ros::NodeHandle nh;
  this->scale = scale;
  this->_pc_pub    = nh.advertise<sensor_msgs::PointCloud2>(point_cloud_node, 1);
  this->_depth_pub = nh.advertise<sensor_msgs::Image>(depth_node, 1);
}

void StereoPipeline::setCalibration(const cares_msgs::StereoCameraInfo stereo_info, double ratio) {
  //Load calibration
  this->_calibration = LoadCalibration(stereo_info, ratio);
  this->_rectificationParameters = nullptr;

  // Show that the calibration was loaded successfully
  if (_calibration->LoadSuccess()) ROS_INFO("Calibration successfully loaded!");
  else ROS_ERROR("Calibration loading failed!");

  // Load the rectification parameters if the calibration was loaded
  if (_calibration->LoadSuccess()){
    _rectificationParameters = OpticsUtils::FindRectification(_calibration);
  }
}

Calibration* StereoPipeline::LoadCalibration(const cares_msgs::StereoCameraInfo stereo_info, double ratio){
  Size imageSize(stereo_info.left_info.width * ratio, stereo_info.left_info.width * ratio);
  cv::Mat K1_tmp(3, 3, CV_64FC1, (void *) stereo_info.left_info.K.data());
  cv::Mat K1 = K1_tmp.clone();
  K1 = K1*ratio;
  ((double *)K1.data)[8] = 1.0;
  cv::Mat K2_tmp(3, 3, CV_64FC1, (void *) stereo_info.right_info.K.data());
  cv::Mat K2 = K2_tmp.clone();
  K2 = K2*ratio;
  ((double *)K2.data)[8] = 1.0;
  cv::Mat D1_tmp(1, 5, CV_64FC1, (void *) stereo_info.left_info.D.data());
  cv::Mat D1 = D1_tmp.clone();
  cv::Mat D2_tmp(1, 5, CV_64FC1, (void *) stereo_info.right_info.D.data());
  cv::Mat D2 = D2_tmp.clone();
  cv::Mat T_tmp(3, 1, CV_64FC1, (void *) stereo_info.T_left_right.data());
  cv::Mat T = T_tmp.clone();
//  T= T*1000.0;
  cv::Mat R_tmp(3, 3, CV_64FC1, (void *) stereo_info.R_left_right.data());
  cv::Mat R = R_tmp.clone();
  // R= R*-1.0;
  // ((double *)R.data)[0] = ((double *)R.data)[0] * -1.0;
  // ((double *)R.data)[4] = ((double *)R.data)[4] * -1.0;
  // ((double *)R.data)[8] = ((double *)R.data)[8] * -1.0;
//  std::cout<<K1<<K2<<D1<<D2<<T<<R<<std::endl;
  return new Calibration(K1, K2, D1, D2, R, T, imageSize);
}

/**
 * Main Terminator
 */ 
StereoPipeline::~StereoPipeline()
{
    delete _calibration;
    if (_rectificationParameters != nullptr) delete _rectificationParameters;
}

//----------------------------------------------------------------------------------
// Image Callback Functionality
//----------------------------------------------------------------------------------

/**
 * Handles the image callback
 * @param image1 The first image passed to the system
 * @param image2 The second image passed to the system
 */
void StereoPipeline::Launch(const sensor_msgs::ImageConstPtr& left_image_msg,
                            const sensor_msgs::ImageConstPtr& right_image_msg,
                            const cares_msgs::StereoCameraInfoConstPtr& stereo_info)
{
  try
  {
      this->setCalibration(*stereo_info, this->scale);
      // Retrieve the raw images
      cv::Mat left_image  = cv_bridge::toCvShare(left_image_msg, "bgr8")->image;
      cv::Mat right_image = cv_bridge::toCvShare(right_image_msg, "bgr8")->image;
      
      //extract the timestamp
      auto timeStamp = left_image_msg->header.stamp;

      // Build a stereo frame
      auto frame = StereoFrame(left_image, right_image);

      // Make a small frame
//      auto smallFrame = StereoFrameUtils::Resize(frame, 1000);

      // Process the stereo frame
//      ProcessFrame(smallFrame, timeStamp);
      ProcessFrame(frame, timeStamp);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("Unable to convert images: %s", e.what());        
  }
  catch (string exception) 
  {
      ROS_ERROR("ERROR: %s", exception.c_str());
  }  
}

//----------------------------------------------------------------------------------
// Main Processing Logic
//----------------------------------------------------------------------------------

/**
 * The main processing pipeline logic goes here
 * @param frame The frame that we are processing
 */
void StereoPipeline::ProcessFrame(StereoFrame& frame, ros::Time timestamp) 
{
    if (!_calibration->LoadSuccess()) return;
    auto uframe = RemoveDistortion(frame);
    auto rframe = PerformRectification(uframe);
    auto dispframe = PerformStereoMatching(rframe);  
    auto depthframe = PerformDepthExtraction(dispframe);
    PublishDepthFrame(depthframe, timestamp);
    DisplayUtils::ShowDepthFrame("Frame", &depthframe, 1000);
    GenerateModel(depthframe, timestamp);
    cv::waitKey(30);
}

void StereoPipeline::PublishDepthFrame(DepthFrame& frame, ros::Time timestamp)
{
    cv_bridge::CvImagePtr cv_ptr;
    try{
      auto header = std_msgs::Header();
      header.stamp= timestamp;
      this->_depth_pub.publish(cv_bridge::CvImage(header, "32FC1", frame.Depth()/1000.0).toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

//--------------------------------------------------
// RemoveDistortion
//--------------------------------------------------

/**
 * Remove distortion from the images
 * @param frame The frame that we are removing distortion from
 * @return The new frame 
 */
StereoFrame StereoPipeline::RemoveDistortion(StereoFrame & frame) 
{
    //ROS_INFO("Removing distortion from the stereo pair");
    auto result = StereoFrameUtils::Undistort(_calibration, frame);
    return result;
}

//--------------------------------------------------
// PerformRectification
//--------------------------------------------------

/**
 * Add the logic to perform the given rectification
 * @param frame The frame that we are rectifying
 * @return The frame that has been rectified  
 */
StereoFrame StereoPipeline::PerformRectification(StereoFrame & frame) 
{
    //ROS_INFO("Performing Rectification");
    auto result = StereoFrameUtils::RectifyFrames(_calibration, _rectificationParameters, frame);
    return result;
}

//--------------------------------------------------
// PerformStereoMatching
//--------------------------------------------------

/**
 * Add the logic to perform stereo matching 
 * @param frame The frame that we are matching
 * @return The depthframe result
 */
DepthFrame StereoPipeline::PerformStereoMatching(StereoFrame & frame) 
{
    //ROS_INFO("Determining disparity ranges");
    //auto disparityMin = Math3D::FindDisparity(_rectificationParameters->GetQ(), MAX_DEPTH, false);
    //auto disparityMax = Math3D::FindDisparity(_rectificationParameters->GetQ(), MIN_DEPTH, true);

    //if (disparityMax == INFINITY || disparityMin == INFINITY) throw string("Either ZMIN or ZMAX (or both) has not been set (or set to zero)");
 
    //ROS_INFO("Disparity Min = %f", disparityMin);
    //ROS_INFO("Disparity Max = %f", disparityMax);

    //ROS_INFO("Performing stereo matching");
    auto matcher = SGBM(0, 16*16);
    cv::Mat disparityMap = matcher.Match(frame);

    return DepthFrame(frame.Image1(), disparityMap);
}

//--------------------------------------------------
// PerformDepthExtraction
//--------------------------------------------------

/**
 * Add the logic to perform depth extraction
 * @param frame The frame that we are extracting the depthframe from
 * @return The resultant depth frame 
 */
DepthFrame StereoPipeline::PerformDepthExtraction(DepthFrame& frame) 
{
    //ROS_INFO("Converting a disparity map into a depth map");
    cv::Mat depthMap = OpticsUtils::ExtractDepthMap(_rectificationParameters->GetQ(), frame.Depth(), MIN_DEPTH, MAX_DEPTH);
    return DepthFrame(frame.Color(), depthMap);
}

//--------------------------------------------------
// PerformDepthExtraction
//--------------------------------------------------

/**
 * Add the functionality to generate a 3D model
 * @param frame The depth frame that we are generating the model for
 */
void StereoPipeline::GenerateModel(DepthFrame& frame,ros::Time timestamp) 
{
    vector<ColorPoint> pointCloud; CloudUtils::ExtractCloud(pointCloud, _rectificationParameters->GetQ(), frame.Color(), frame.Depth());

    // use the current pose for now!
    //CloudUtils::TransformCloud(_pose, pointCloud);
    sensor_msgs::PointCloud2 pc_msgs;
    pc_msgs.header.frame_id = "stereo1/left";
    pc_msgs.header.stamp = timestamp;
    pc_msgs.height = 1;
    pc_msgs.width = pointCloud.size();
    sensor_msgs::PointCloud2Modifier modifier(pc_msgs);
    std::cout<<"this is pointcloud"<<pc_msgs<<std::endl;
    modifier.setPointCloud2Fields(6, "x", 1, sensor_msgs::PointField::FLOAT32,
                                              "y", 1, sensor_msgs::PointField::FLOAT32,
                                              "z", 1, sensor_msgs::PointField::FLOAT32,
                                              "r", 1, sensor_msgs::PointField::UINT8,
                                              "g", 1, sensor_msgs::PointField::UINT8,
                                              "b", 1, sensor_msgs::PointField::UINT8);
    modifier.setPointCloud2FieldsByString(2, "xyz","rgb");
    sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msgs, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msgs, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msgs, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pc_msgs, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pc_msgs, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pc_msgs, "b");
    pc_msgs.is_bigendian = false;
    pc_msgs.is_dense = true;
    for(ColorPoint p : pointCloud){
        iter_x[0] = p.GetLocation().x/1000.0;
        iter_y[0] = p.GetLocation().y/1000.0;
        iter_z[0] = p.GetLocation().z/1000.0;
        iter_r[0] = p.GetColor()[0];
        iter_g[0] = p.GetColor()[1];
        iter_b[0] = p.GetColor()[2];
        //std::cout<<iter_r[0]<<iter_g[0]<<iter_b[0]<<endl;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
    }
    this->_pc_pub.publish(pc_msgs);
}

unsigned int StereoPipeline::cvtRGB(Vec3i _color){
   uint out =0;
   out = (_color[0])+((_color[1])<<8)+(_color[2]<<16);
   return out; 
}
