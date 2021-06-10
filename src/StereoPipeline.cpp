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
 * @brief Main constructor
 * @param cloudNodeName The name of the ros_node holding the point cloud
 * @param depthNodeName The name of the ros_node holding the depth information
 * @param scale The scaling factor that we want to scale the images by
 */
StereoPipeline::StereoPipeline(double scale)
{
  ros::NodeHandle nh;
  _imageColorPublisher = nh.advertise<sensor_msgs::Image>("stereo/image_color", 1);
  _cameraInfoPublisher = nh.advertise<sensor_msgs::CameraInfo>("stereo/camera_info", 1);
  _depthPublisher      = nh.advertise<sensor_msgs::Image>("stereo/depth", 1);
  _pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("stereo/points", 1);

  _scale = scale;
  _rectificationParameters = nullptr;  
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
 * @param stereo_info The incoming stereo information
 */
void StereoPipeline::Launch(const sensor_msgs::ImageConstPtr& left_image_msg,
                            const sensor_msgs::ImageConstPtr& right_image_msg,
                            const cares_msgs::StereoCameraInfoConstPtr& stereo_info)
{
  try
  {
      // Set the calibration based on the incoming stereo info
      ros::Duration end;
      double ns_to_ms = 1000000.0;
      ros::Time start_calibration = ros::Time::now();

      SetCalibration(*stereo_info, _scale);

      end = ros::Time::now() - start_calibration;
      ROS_INFO("Calibration Setup: %f", end.nsec/ns_to_ms);

      // Retrieve the raw images
      cv::Mat left_image  = cv_bridge::toCvShare(left_image_msg, "bgr8")->image;
      cv::Mat right_image = cv_bridge::toCvShare(right_image_msg, "bgr8")->image;

      // Build a stereo frame
      ros::Time start_frame = ros::Time::now();
      
      auto frame = StereoFrame(left_image, right_image);
      
      end = ros::Time::now() - start_frame;
      ROS_INFO("Frame Setup: %f", end.nsec/ns_to_ms);

      // NOTE: Enable and use this if we want to use the scaling factor
      //auto smallFrame = StereoFrameUtils::Resize(frame, 1000);

      // Launch the stereo processing pipeline to process the frame
      ros::Time start_process = ros::Time::now();
      
      std_msgs::Header header = left_image_msg->header;
      ProcessFrame(frame, header);
      
      sensor_msgs::CameraInfo camera_info = generateCameraInfo(header);
      _cameraInfoPublisher.publish(camera_info);

      end = ros::Time::now() - start_process;
      ROS_INFO("Process: %f", end.nsec/ns_to_ms);

      end = ros::Time::now() - start_calibration;
      ROS_INFO("Total: %f", end.nsec/ns_to_ms);
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
// Load the calibration
//----------------------------------------------------------------------------------

sensor_msgs::CameraInfo StereoPipeline::generateCameraInfo(const std_msgs::Header& header) {
    sensor_msgs::CameraInfo camera_info;
    camera_info.header = header;
    camera_info.width  = this->_calibration->GetImageSize().width;
    camera_info.height = this->_calibration->GetImageSize().height;
    camera_info.roi.do_rectify = false;

    cv::Mat P1 = this->_rectificationParameters->GetP1();
    std::vector<double>P_v(P1.begin<double>(), P1.end<double>());

    //Intrinsic parameters
    camera_info.K = {P_v[0], P_v[1], P_v[2], P_v[4], P_v[5], P_v[6], P_v[8], P_v[9], P_v[10]};
    camera_info.D = {0,0,0,0,0};
    for (int i=0; i<P_v.size(); i++)camera_info.P[i] = (P_v[i]);
    camera_info.R = {1,0,0,0,1,0,0,0,1};
    return camera_info;
}

/**
 * @brief Set the calibration from the ROS message
 * @param stereo_info The stereo information
 * @param ratio The scaling factor
 */
void StereoPipeline::SetCalibration(const cares_msgs::StereoCameraInfo & stereo_info, double ratio)
{
    LoadCalibration(stereo_info, ratio);
}

/**
 * @brief Calibration loading helper
 * @param stereo_info The incoming stereo information
 * @param ratio The scaling factor
 * @return The loaded stereo information
 */
Calibration* StereoPipeline::LoadCalibration(const cares_msgs::StereoCameraInfo& stereo_info, double ratio)
{
    // Determine the image size
     Size imageSize(stereo_info.left_info.width, stereo_info.left_info.height);

    // Load and create the camera matrix for the left camera
    cv::Mat K1_tmp(3, 3, CV_64FC1, (void *) stereo_info.left_info.K.data());
    cv::Mat K1 = K1_tmp.clone();
    //K1 = K1 * ratio;
    //((double *)K1.data)[8] = 1.0;

    // Load and create the camera matrix for the right camera
    cv::Mat K2_tmp(3, 3, CV_64FC1, (void *) stereo_info.right_info.K.data());
    cv::Mat K2 = K2_tmp.clone();
    //K2 = K2 * ratio;
    //((double *)K2.data)[8] = 1.0;

    // Load the distortion for the left camera
    cv::Mat D1_tmp(1, stereo_info.left_info.D.size(), CV_64FC1, (void *) stereo_info.left_info.D.data());
    cv::Mat D1 = D1_tmp.clone();

    // Load the distortion for the right camera
    cv::Mat D2_tmp(1, stereo_info.right_info.D.size(), CV_64FC1, (void *) stereo_info.right_info.D.data());
    cv::Mat D2 = D2_tmp.clone();

    // Load the translation
    cv::Mat T_tmp(3, 1, CV_64FC1, (void *) stereo_info.T_left_right.data());
    cv::Mat T = T_tmp.clone();

    // Load the rotation
    cv::Mat R_tmp(3, 3, CV_64FC1, (void *) stereo_info.R_left_right.data());
    cv::Mat R = R_tmp.clone();

    // Return the calibration details
    this->_calibration = new Calibration( K1, K2, D1, D2, R, T, imageSize);

    cv::Mat R1(3, 3, CV_64FC1, (void *)stereo_info.left_info.R.data());
    cv::Mat P1(3, 4, CV_64FC1, (void *)stereo_info.left_info.P.data());

    cv::Mat R2(3, 3, CV_64FC1, (void *)stereo_info.right_info.R.data());
    cv::Mat P2(3, 4, CV_64FC1, (void *)stereo_info.right_info.P.data());

    cv::Mat Q(4, 4, CV_64FC1, (void *)stereo_info.Q.data());
    this->_rectificationParameters = new RectificationParameters(R1, R2, P1, P2, Q);
}

//----------------------------------------------------------------------------------
// Processing Logic
//----------------------------------------------------------------------------------

/**
 * The main processing pipeline logic goes here
 * @param frame The frame that we are processing
 * @param header The current ROS header for saving the frame
 */
void StereoPipeline::ProcessFrame(StereoFrame& frame, std_msgs::Header& header)
{
    // Confirm that the calibration was loaded succesfully
    if (!_calibration->LoadSuccess()) throw "Calibration was not loaded succesfully";

    // Remove the distortion from the frame
//    auto uframe = RemoveDistortion(frame);

    // Rectify the frame
//    auto rframe = PerformRectification(uframe);

    // Calculate a disparity map for the frame
    auto dispFrame = PerformStereoMatching(frame);
    DisplayUtils::ShowDepthFrame("Disparity", &dispFrame, 1000);

    // Convert the disparity map into a depth map
    auto depthFrame = PerformDepthExtraction(dispFrame);

    // Show the depth frame for debug purposes
    Mat depth = depthFrame.Depth() * 2;
    auto scaledDepth = DepthFrame(depthFrame.Color(), depth);
    DisplayUtils::ShowDepthFrame("Frame", &scaledDepth, 1000);
    cv::waitKey(30);

    // Publish the depth frame to ROS
    PublishDepthFrame(depthFrame, header);

    // Generate the point cloud and publish it to ROS
    GenerateAndPublishPointCloud(depthFrame, header);    
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
    ROS_INFO("Removing distortion from the stereo pair");
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
    ROS_INFO("Performing Rectification");
    auto result = StereoFrameUtils::RectifyFrames(_calibration, _rectificationParameters, frame);
    return result;
}

//--------------------------------------------------
// PerformStereoMatching
//--------------------------------------------------

/**
 * WARNING: THE DISPARITY RANGE HAS BEEN HARD CODED!!!!
 * Add the logic to perform stereo matching 
 * @param frame The frame that we are matching
 * @return The depth frame result
 */
DepthFrame StereoPipeline::PerformStereoMatching(StereoFrame & frame) 
{
    ROS_INFO("Performing stereo matching");
    auto matcher = SGBM(0, 16 * 32);
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
    ROS_INFO("Converting a disparity map into a depth map");
    cv::Mat depthMap = OpticsUtils::ExtractDepthMap(_rectificationParameters->GetQ(), frame.Depth(), 0, 1e10);
    return DepthFrame(frame.Color(), depthMap);
}

//--------------------------------------------------
// PublishDepthFrame
//--------------------------------------------------

/**
 * Publish the depth frame to ROS
 * @param frame The depthframe that we are publishing
 * @param header The header that we are publishing
 */
void StereoPipeline::PublishDepthFrame(DepthFrame& frame, std_msgs::Header& header)
{
  try
  {
    ROS_INFO("Publishing the ROS depth frame");
    _depthPublisher.publish(cv_bridge::CvImage(header, "32FC1", frame.Depth()).toImageMsg());
    _imageColorPublisher.publish(cv_bridge::CvImage(header, "bgr8", frame.Color()).toImageMsg());
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}
//--------------------------------------------------
// Generate and Publish Point Cloud
//--------------------------------------------------

/**
 * @brief Generate and publish the point cloud to ROS
 * @param frame The depth frame we are generating the point cloud from
 * @param header The corresponding header
 */
void StereoPipeline::GenerateAndPublishPointCloud(DepthFrame& frame, std_msgs::Header& header)
{
    // Generate the point cloud
    vector<ColorPoint> pointCloud; CloudUtils::ExtractCloud(pointCloud, _rectificationParameters->GetQ(), frame.Color(), frame.Depth());

    // Create the header for the point cloud message
    sensor_msgs::PointCloud2 pc_msgs;
    pc_msgs.header = header;
    pc_msgs.height = 1;
    pc_msgs.width = pointCloud.size();

//    cout << "This is point cloud: " << pc_msgs << endl;

    // Insert the point cloud into the message
    sensor_msgs::PointCloud2Modifier modifier(pc_msgs);
    modifier.setPointCloud2Fields(6,    "x", 1, sensor_msgs::PointField::FLOAT32,
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

    // Copy across the point cloud data
    for(auto point : pointCloud)
    {
        iter_x[0] = point.GetLocation().x;
        iter_y[0] = point.GetLocation().y;
        iter_z[0] = point.GetLocation().z;
        iter_r[0] = point.GetColor()[0];
        iter_g[0] = point.GetColor()[1];
        iter_b[0] = point.GetColor()[2];

       // update iterators
        ++iter_x; ++iter_y; ++iter_z; ++iter_r; ++iter_g; ++iter_b;
    }

    // Publish the result
    _pointCloudPublisher.publish(pc_msgs);
}
