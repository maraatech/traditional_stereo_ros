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
StereoPipeline::StereoPipeline(ros::NodeHandle& nh) 
{
    ROS_INFO("START!");
    // Load calibration
    auto calibrationPath = GeneralUtils::CompletePath(CALIBRATION_PATH_JSON);
    //_calibration = LoadUtils::LoadCalibration(calibrationPath);
    // _calibration = NULL;
    ros::Subscriber s = _nh.subscribe<maara_msgs::StereoCameraInfo>("/rectified_image/stereo_info",1,&StereoPipeline::LoadCalibrationCallback,this);
    ros::Rate loop_rate(0.5);
    while(this->_calibration == NULL){
        ROS_INFO("waiting for info");
        ros::spinOnce();
        loop_rate.sleep();
    }
    loop_rate.sleep();
    _rectificationParameters = nullptr;
    this->_nh = nh;
    this->_pc_pub = this->_nh.advertise<sensor_msgs::PointCloud2>("stereo/points", 1);
    this->_depth_pub = this->_nh.advertise<sensor_msgs::Image>("stereo/depth", 1);
   
    // Show that the calibration was loaded successfully
    if (_calibration->LoadSuccess()) ROS_INFO("Calibration successfully loaded!");
    else ROS_WARN("Calibration loading failed!");

    // Load the rectification parameters if the calibraiton was loaded
    if (_calibration->LoadSuccess()) 
    {
        _rectificationParameters = OpticsUtils::FindRectification(_calibration);
        auto path = stringstream(); path << IMAGE_FOLDER << "rectification.xml";
        std::cout<<path.str()<<std::endl;
        OpticsUtils::SaveRectificationParameters(path.str(), *_rectificationParameters);
    }
}

void StereoPipeline::LoadCalibrationCallback(const maara_msgs::StereoCameraInfo ci){
    ROS_INFO("GOT INFO!");
    double RATIO = 1000.0/ci.left_info.width;
    this->_calibration = LoadUtils::LoadCalibration(ci,RATIO);
    ROS_INFO("DONE!");
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
void StereoPipeline::Launch(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2)
{
  try
  {
      // Retrieve the raw images
      cv::Mat rawImage1 = cv_bridge::toCvShare(image1, "bgr8")->image;
      cv::Mat rawImage2 = cv_bridge::toCvShare(image2, "bgr8")->image;
      
      //extract the timestamp
      auto timeStamp = image1->header.stamp;

      // Build a stereo frame
      auto frame = StereoFrame(rawImage1, rawImage2);

      // Make a small frame
      auto smallFrame = StereoFrameUtils::Resize(frame, 1000);

      // Process the stereo frame
      ProcessFrame(smallFrame, timeStamp);  
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
    try
    {
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
    // Save the Cloud to scans
    //auto savePath = stringstream(); savePath << GeneralUtils::CompletePath(IMAGE_FOLDER) << GeneralUtils::GetTimeString() << ".ply";
    //auto cloudWriter = PointSaver(); cloudWriter.AddPointSet(pointCloud);
    //cloudWriter.Save(savePath.str());
}

unsigned int StereoPipeline::cvtRGB(Vec3i _color){
   uint out =0;
   out = (_color[0])+((_color[1])<<8)+(_color[2]<<16);
   return out; 
}  