//
// Created by rain on 17-9-18.
//

#ifndef RAIN_VIO_SYSTEM_H
#define RAIN_VIO_SYSTEM_H

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <opencv2/core/core.hpp>

#include "../include /Tracking.h"

using namespace std;

namespace RAIN_VIO
{

class Tracking;

class System
{

private:

    Tracking *mTracker;

    std::thread* mptLocalMapping;
    std::thread* mptViewer;
//    std::thread* mptLoopClosing;
public:

    System(const string &strSettingsFile, const bool bOpenViwer);

    void TrackMono(const cv::Mat &image, const double &TimeStamps);

public:

    // camera

};


}




#endif //RAIN_VIO_SYSTEM_H
