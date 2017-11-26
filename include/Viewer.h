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
class MapDrawer;

class Viewer
{

public:
    Viewer(const string &strSettingPath, MapDrawer* pMapDrawer);

    void Run();

    void UpdateFrame(Tracking *pTracking);

private:

    Tracking *mpTracking;
    MapDrawer *mpMapDrawer;

    vector<cv::Point2f> mvFraPointsPts;
    vector<uint> mvFraPointsID;
    vector<int> mvFraPointsCnt;

    cv::Mat mframe;
    cv::Mat mCurImageShow;
    double mImageHeight;
    double mImageWidth;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    mutex mMutex;

}; // class Viewer

} // namesapce RAIN_VIO

#endif //RAIN_VIO_VIEWER_H
