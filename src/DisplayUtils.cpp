//--------------------------------------------------
// Implementation code for DisplayUtils
//
// @author: Wild Boar
//--------------------------------------------------

#include "DisplayUtils.h"
using namespace Amantis;

//--------------------------------------------------
// ShowDepthFrame
//--------------------------------------------------

/**
 * Display a stereo frame frame on the screen
 * @param caption The caption associated with the frame
 * @param frame The frame that we are displaying
 * @param width The width of the frame that we are displaying
 */
void DisplayUtils::ShowStereoFrame(const string& caption, StereoFrame* frame, int width)
{
	int actualWidth = frame->Image1().cols + frame->Image2().cols;
	auto factor = (double)width / (double)actualWidth;

	cv::Mat result = cv::Mat::zeros(frame->Image1().rows, actualWidth, CV_8UC3);
	int height1 = frame->Image1().rows; int width1 = frame->Image1().cols;
	frame->Image1().copyTo(result( Range(0,height1), Range(0, width1) ));

	int height2 = frame->Image2().rows; int width2 = frame->Image2().cols;
	frame->Image2().copyTo(result(Range(0, height2), Range(width1, width1 + width2)));

	Mat smallImage; cv::resize(result, smallImage, Size(), factor, factor);

	cv::imshow(caption, smallImage);
}

