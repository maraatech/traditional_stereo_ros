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