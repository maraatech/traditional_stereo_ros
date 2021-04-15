//--------------------------------------------------
// Implementation code for GeneralUtils
//
// @author: Wild Boar
//--------------------------------------------------

#include "GeneralUtils.h"
using namespace Amantis;  

//----------------------------------------------------------------------------------
// GetTimeString
//----------------------------------------------------------------------------------

/**
 * Retrieves a given time string
 * @return Retrieve the given time string
 */
string GeneralUtils::GetTimeString()
{
    // get current time
    auto now = chrono::system_clock::now();

    // get number of milliseconds for the current second
    // (remainder after division into seconds)
    auto ms = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;

    // convert to std::time_t in order to convert to std::tm (broken time)
    auto timer = chrono::system_clock::to_time_t(now);

    // convert to broken time
    std::tm bt = *std::localtime(&timer);

    std::ostringstream oss;

    oss << std::put_time(&bt, "%H_%M_%S_"); 
    oss << std::setfill('0') << std::setw(3) << ms.count();

    return oss.str();
}

//----------------------------------------------------------------------------------
// GetTimeString
//----------------------------------------------------------------------------------

/**
 * This is a linux based functionality for adding the location of the home directory at the beginning of the path
 * @param path The path that we are completing
 * @return The resultant path
 */
string GeneralUtils::CompletePath(const string& path) 
{
    string homeFolder = getenv("HOME");
    auto fullPath = stringstream(); fullPath << homeFolder << path;
    return fullPath.str();
}
