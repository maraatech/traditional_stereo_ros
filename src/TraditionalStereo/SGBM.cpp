//--------------------_optical_frame------------------------------
// Implementation code for SGBM
//
// @author: Wild Boar
//--------------------------------------------------

#include "SGBM.h"
using namespace Amantis;

//--------------------------------------------------
// Constructors
//--------------------------------------------------

/**
 * Main Constructor
 * @param minDisparity The min disparity (must be a factor of 16)
 * @param maxDisparity The max disparity (must be a factor of 16)
 */
SGBM::SGBM(int minDisparity, int maxDisparity)
{
	_minDisparity = minDisparity; _maxDisparity = maxDisparity;
	auto p1 = 8 * 1 * SGBM_BLOCK_SIZE * SGBM_BLOCK_SIZE; auto p2 = 32 * 1 * SGBM_BLOCK_SIZE * SGBM_BLOCK_SIZE;
	_matcher = StereoSGBM::create(minDisparity, maxDisparity - minDisparity, SGBM_BLOCK_SIZE, p1, p2, SGBM_DISP_12_MAX_DIFF, SGBM_PRE_FILTER_CAP, SGBM_UNIQUENESS_RATIO, SGBM_SPECKLE_WINDOW_SIZE, SGBM_SPECKLE_RANGE, StereoSGBM::MODE_SGBM);
}

//--------------------------------------------------
// Matching Logic
//--------------------------------------------------

/**
 * Perform stereo matching
 * @param frame The frame that we are matching against
 * @return Return a Mat
 */
Mat SGBM::Match(StereoFrame & frame)
{
	// Convert to grayscale (more robust than color in theory)
	Mat gray1; cvtColor(frame.Image1(), gray1, COLOR_BGR2GRAY);
	Mat gray2; cvtColor(frame.Image2(), gray2, COLOR_BGR2GRAY);

	// Noise removal using guassian smoothing
	GaussianBlur(gray1, gray1, Size(3, 3), 1, 1);
	GaussianBlur(gray2, gray2, Size(3, 3), 1, 1);

	// Equalization to deal with color differences
	equalizeHist(gray1, gray1); equalizeHist(gray2, gray2);

	// Perform the stereo matching
	Mat disparity; _matcher->compute(gray1, gray2, disparity);

	// Build the result
	return ConvertResult(disparity);
}

//--------------------------------------------------
// Convert result
//--------------------------------------------------

/**
 * Make sure that the result is a double map that has not been scaled by 16
 * @param disparity the map that we are scaling
 */
Mat SGBM::ConvertResult(Mat& disparity) 
{
	Mat result = Mat_<float>::zeros(disparity.size());

	auto input = (ushort*)disparity.data;
	auto output = (float*)result.data;

	for (auto row = 0; row < result.rows; row++) 
	{
		for (auto column = 0; column < result.cols; column++) 
		{
			auto index = column + row * result.cols;

			auto disparityValue = input[index] / 16.0F;
		
			//if (disparityValue < _minDisparity || disparityValue > _maxDisparity) continue;

			output[index] = disparityValue;
		}	
	}

	return result;
}
