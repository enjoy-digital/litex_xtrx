/**
@file VersionInfo.in.cpp
@author Lime Microsystems
@brief API for querying version and build information.
*/

#include "lms7002mNG/VersionInfo.h"
#include <sstream>

#define QUOTE_(x) #x
#define QUOTE(x) QUOTE_(x)

using namespace std::literals::string_literals;

std::string lime::GetLibraryVersion(void)
{
    return "@LIME_SUITE_VERSION@"s;
}

std::string lime::GetBuildTimestamp(void)
{
    return "@BUILD_TIMESTAMP@"s;
}

std::string lime::GetAPIVersion(void)
{
    const std::string verStr(QUOTE(LIMESUITENG_API_VERSION));
    std::stringstream ss;
    ss << std::stoi(verStr.substr(2, 4)) << "." << std::stoi(verStr.substr(6, 2)) << "." << std::stoi(verStr.substr(8, 2));
    return ss.str();
}

std::string lime::GetABIVersion(void)
{
    return "@LIME_SUITE_SOVER@"s;
}
