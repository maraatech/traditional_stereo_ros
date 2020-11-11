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
StereoPipeline::StereoPipeline()
{
    // Load calibration
    auto calibrationPath = GeneralUtils::CompletePath(CALIBRATION_PATH);
    _calibration = LoadUtils::LoadCalibration(calibrationPath);
    _rectificationParameters = nullptr;

    // Show that the calibration was loaded successfully
    if (_calibration->LoadSuccess()) ROS_INFO("Calibration successfully loaded!");
    else ROS_WARN("Calibration loading failed!");

    // Load the rectification parameters if the calibraiton was loaded
    if (_calibration->LoadSuccess()) _rectificationParameters = OpticsUtils::FindRectification(_calibration);
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

      // Build a stereo frame
      auto frame = StereoFrame(rawImage1, rawImage2);

      // Make a small frame
      auto smallFrame = StereoFrameUtils::Resize(frame, 1000);

      // Process the stereo frame
      ProcessFrame(smallFrame);  
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
void StereoPipeline::ProcessFrame(StereoFrame& frame) 
{
    if (!_calibration->LoadSuccess()) return;

    auto uframe = RemoveDistortion(frame);
    auto rframe = PerformRectification(uframe);
    auto dispframe = PerformStereoMatching(rframe);  
    auto depthframe = PerformDepthExtraction(dispframe);

    DisplayUtils::ShowDepthFrame("Frame", &depthframe, 1000);
    cv::waitKey(30);
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

