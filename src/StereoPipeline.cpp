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
    // TODO: Implement this
}

/**
 * Main Terminator
 */ 
StereoPipeline::~StereoPipeline()
{
    // TODO: Implement this
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
      cv::Mat imageData1 = cv_bridge::toCvShare(image1, "bgr8")->image;
      cv::Mat imageData2 = cv_bridge::toCvShare(image2, "bgr8")->image;

      // Build a stereo frame
      auto frame = StereoFrame(imageData1, imageData2);

      // Show the stereo frame on the screen
      DisplayUtils::ShowStereoFrame("Frame", &frame, 1000);

      // Create a small version of the frame
      auto smallFrame = StereoFrameUtils::Resize(frame, 1000);

      // Save the frame to disk
      string suffix = GeneralUtils::GetTimeString();
      auto baseFolder = GeneralUtils::CompletePath(IMAGE_FOLDER);
      StereoFrameUtils::Save(smallFrame, baseFolder, suffix, ".jpg");

      // Pause the screen
      cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("Unable to show images on the screen");
  }
}
