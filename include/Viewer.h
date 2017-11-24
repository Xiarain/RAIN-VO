//
// Created by rain on 17-10-21.
//

#ifndef RAIN_VIO_VIEWER_H
#define RAIN_VIO_VIEWER_H

#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <glog/logging.h>

#include "Tracking.h"
#include "Parameters.h"


using namespace std;

namespace RAIN_VIO
{

class Tracking;

class Viewer
{

public:
    Viewer(const string &strSettingPath);

    void Run();

    void UpdateFrame(Tracking *pTracking);

private:

    Tracking *mpTracking;

    vector<cv::Point2f> mvFraPointsPts;
    vector<uint> mvFraPointsID;
    vector<int> mvFraPointsCnt;

    cv::Mat mframe;
    cv::Mat mCurImageShow;
    double mImageHeight;
    double mImageWidth;
    const double mFOCALLENGTH = 460.0;

    mutex mMutex;

}; // class Viewer

} // namesapce RAIN_VIO

#endif //RAIN_VIO_VIEWER_H
