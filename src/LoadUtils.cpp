//--------------------------------------------------
// Implementation code for LoadUtils
//
// @author: Wild Boar
//--------------------------------------------------

#include "LoadUtils.h"
using namespace Amantis;

//--------------------------------------------------
// LoadCalibration
//--------------------------------------------------

/**
 * Load calibration from disk
 * @param path The path to the calibration file
 * @return resulting intrinsics
 */
Calibration* LoadUtils::LoadCalibration(const string& path) 
{
	auto reader = FileStorage(path, FileStorage::READ | FileStorage::FORMAT_XML);
	
	Mat K1; reader["Camera1"] >> K1; 
	Mat K2; reader["Camera2"] >> K2;
	Mat D1; reader["Distortion1"] >> D1;
	Mat D2; reader["Distortion2"] >> D2;
	Mat R; reader["Rotation"] >> R;
	Mat T; reader["Translation"] >> T;
	
	return new Calibration(K1, K2, D1, D2, R, T);
}

