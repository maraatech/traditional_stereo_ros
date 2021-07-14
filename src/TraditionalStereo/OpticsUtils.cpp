//--------------------------------------------------
// Implementation code for OpticsUtils
//
// @author: Wild Boar
//--------------------------------------------------

#include "OpticsUtils.h"
using namespace Amantis;

//--------------------------------------------------
// FindRectification
//--------------------------------------------------

/**
 * Defines the logic to find the given rectification
 * @param calibration The current calibration parameters
 * @return The rectification parameters that were found
 */
RectificationParameters* OpticsUtils::FindRectification(Calibration* calibration)
{
	Mat noDistortion = Mat_<double>::zeros(4, 1);
	Mat R1, R2, P1, P2, Q;
	stereoRectify(calibration->GetCamera1(), calibration->GetDistortion1(), calibration->GetCamera2(), calibration->GetDistortion2(), calibration->GetImageSize(), calibration->GetRotation(), calibration->GetTranslation(), R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, calibration->GetImageSize());
	return new RectificationParameters(R1, R2, P1, P2, Q);
}

//--------------------------------------------------
// ExtractDepthMap
//--------------------------------------------------

/**
 * Extract a depth map from a disparity map and Q Matrix
 * @param Q The given Q Matrix
 * @param disparityMap The disparity map that we are presenting
 * @param zmin The minimum Z value
 * @param zmax The maximum Z value
 */
Mat OpticsUtils::ExtractDepthMap(Mat& Q, Mat& disparityMap, double zmin, double zmax)
{
	auto qdata = (double*)Q.data;

	auto f = qdata[11];
	auto baseline = (-1.0 / qdata[14]);
	if (baseline < 0) baseline *= -1; // Make sure that the baseline is always positive

	Mat result = Mat_<double>::zeros(disparityMap.size());

	auto input = (double*)disparityMap.data;
	auto output = (double*)result.data;

	for (auto row = 0; row < disparityMap.rows; row++) 
	{
		for (auto column = 0; column < disparityMap.cols; column++) 
		{
			auto index = column + row * disparityMap.cols;

			auto disparityValue = input[index];
			auto Z = (f * baseline) / disparityValue;

			if (Z < zmin || Z > zmax) continue;

			output[index] = Z;
		}	
	}

	return result;
}

//--------------------------------------------------
// ExtractDepthMap
//--------------------------------------------------

/**
 * A utility for the saving of rectification parameters to disk
 * @param path The path that we are saving to
 * @param parameters The parameters that we are saving
 */
void OpticsUtils::SaveRectificationParameters(const string & path, RectificationParameters& parameters) 
{
	auto writer = cv::FileStorage(path, FileStorage::FORMAT_XML | FileStorage::WRITE);

	writer << "R1" << parameters.GetR1();
	writer << "R2" << parameters.GetR2();
	writer << "P1" << parameters.GetP1();
	writer << "P2" << parameters.GetP2();
	writer << "Q" << parameters.GetQ();

	writer.release();
}
