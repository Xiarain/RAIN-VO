//
// Created by rain on 17-9-19.
//

#ifndef RAIN_VIO_TRACKING_H
#define RAIN_VIO_TRACKING_H

#include <string>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "Frame.h"
#include "Feature.h"
#include "Map.h"

using namespace std;

namespace RAIN_VIO
{

class Camera;
class Frame;
class Map;

class Tracking
{

public:

    Camera *mpcamera;
    Map *mpMap;
    enum eTrackingState {
        NO_INITIALIZED = 0,
        OK = 1,
        LOST = 2,
        BAD = 3
    };

    cv::Mat mMask;
    cv::Mat mImage;
    cv::Mat mNextImage;
    cv::Mat mPreImage;
    cv::Mat mCurImage;
    cv::Mat mNextImageShow, mCurImageShow;

    bool EQUALIZE;
    cv::Mat CmaeraK;
    cv::Mat DistCoef;
    double ImageHeight;
    double ImageWidth;
    double ImageGridHeight;
    double ImageGridWidth;

    int mdFrameCount;

    Tracking(const string &strSettingsFile);
    Tracking();
    ~Tracking();
    void Track(const cv::Mat &image, const double &TimeStamps);

    Frame *mCurrentFrame;
    Feature *feature;

private:

    bool mbFirstImage;
    double mFirstImageTime;
    int numFeatures;
    int minDist;
    uint mIDcnt;
    string mstrSettingsFile;


}; // class Tracking

} // namespce RAIN_VIO

#endif //RAIN_VIO_TRACKING_H
