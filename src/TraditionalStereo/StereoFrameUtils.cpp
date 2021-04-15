//--------------------------------------------------
// Implementation code for StereoFrameUtils
//
// @author: Wild Boar
//--------------------------------------------------

#include "StereoFrameUtils.h"  
using namespace Amantis;

//--------------------------------------------------
// Resize
//--------------------------------------------------

/**
 * Defines the logic to resize a stereo frame
 * @param frame The frame that we are resizing
 * @param imageWidth The width of a single image
 * @return The frame that was returned
 */
StereoFrame StereoFrameUtils::Resize(StereoFrame& frame, int imageWidth) 
{
	auto factor = (double)imageWidth / (double)frame.Width();

    cv::Mat image1; cv::resize(frame.Image1(), image1, cv::Size(), factor, factor);
    cv::Mat image2; cv::resize(frame.Image2(), image2, cv::Size(), factor, factor);    

    return StereoFrame(image1, image2);
}

//--------------------------------------------------
// Save
//--------------------------------------------------

/**
 * Save a stereo frame to disk
 * @param frame The frame that we are saving
 * @param baseFolder The folder that we are saving the frame to
 * @param suffix A value we are adding to the filename to make it unique
 * @param extension What type of image we are saving
 */
void StereoFrameUtils::Save(StereoFrame& frame, const string& baseFolder, const string& suffix, const string& extension) 
{
    auto path1 = stringstream(); path1 << baseFolder << "/Left_" << suffix << extension;
    auto path2 = stringstream(); path2 << baseFolder << "/Right_" << suffix << extension; 

    cv::imwrite(path1.str(), frame.Image1()); cv::imwrite(path2.str(), frame.Image2());
}

//--------------------------------------------------
// UndistortStereoFrame
//--------------------------------------------------

/**
 * Undistort a stereo frame
 * @param frame The depth frame that we are undistorting
 * @param cameraMatrix The camera matrix
 * @param distortion The distortion coefficients
 * @return Return a Mat
 */
StereoFrame StereoFrameUtils::Undistort(Calibration* calibration, StereoFrame& frame)
{
	cv::Mat image1; undistort(frame.Image1(), image1, calibration->GetCamera1(), calibration->GetDistortion1(), calibration->GetCamera1());
	Mat image2; undistort(frame.Image2(), image2, calibration->GetCamera2(), calibration->GetDistortion2(), calibration->GetCamera2());
	return StereoFrame(image1, image2);
}

//--------------------------------------------------
// RectifyFrames
//--------------------------------------------------

/**
 * The main logic to rectify the given set of stereo frames
 * @param calibration The original calibration parameters
 * @param parameters The rectification parameters associated with the application
 * @param frame The frame that we have rectified
 * @return The rectified stereo frame
 */
StereoFrame StereoFrameUtils::RectifyFrames(Calibration * calibration, RectificationParameters* parameters, StereoFrame& frame) 
{
	Mat noDistortion = Mat_<double>::zeros(4, 1);

	// Rectify the first image
	Mat map1X, map1Y; initUndistortRectifyMap(calibration->GetCamera1(), noDistortion, parameters->GetR1(), parameters->GetP1(), calibration->GetImageSize(), CV_32FC1, map1X, map1Y);
	Mat rImage1; remap(frame.Image1(), rImage1, map1X, map1Y, INTER_CUBIC);

	// Rectify the second image
	Mat map2X, map2Y; initUndistortRectifyMap(calibration->GetCamera2(), noDistortion, parameters->GetR2(), parameters->GetP2(), calibration->GetImageSize(), CV_32FC1, map2X, map2Y);
	Mat rImage2; remap(frame.Image2(), rImage2, map2X, map2Y, INTER_CUBIC);

	return StereoFrame(rImage1, rImage2);
}
