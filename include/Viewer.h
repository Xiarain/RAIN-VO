//
// Created by rain on 17-10-21.
//

#ifndef RAIN_VIO_VIEWER_H
#define RAIN_VIO_VIEWER_H

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;

namespace RAIN_VIO
{

class Viewer
{

public:
    Viewer(const string &strSettingPath);
    void Run();

}; // class Viewer

} // namesapce RAIN_VIO

#endif //RAIN_VIO_VIEWER_H
