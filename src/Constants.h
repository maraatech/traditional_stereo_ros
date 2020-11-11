
//--------------------------------------------------
// A module for holding constants and helper functions for the application
// NOTE: THIS SHOULD BE REPLACED BY PARAMETERS
//
// @author: Wild Boar
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

//--------------------------------------------------
// Path Locations
//--------------------------------------------------
constexpr auto IMAGE_FOLDER = "/scans/";
constexpr auto CALIBRATION_PATH = "/Trevor/Maaratech/Data/Calibration/calibration.xml";

//--------------------------------------------------
// Disparity Range parameters
//--------------------------------------------------
constexpr auto MIN_DEPTH = 100;
constexpr auto MAX_DEPTH = 7000;

//--------------------------------------------------
// SGBM PARAMETERS
//--------------------------------------------------
constexpr auto SGBM_BLOCK_SIZE = 7;
constexpr auto SGBM_DISP_12_MAX_DIFF = 0;
constexpr auto SGBM_PRE_FILTER_CAP = 63;
constexpr auto SGBM_UNIQUENESS_RATIO = 9;
constexpr auto SGBM_SPECKLE_WINDOW_SIZE = 200;
constexpr auto SGBM_SPECKLE_RANGE = 1;