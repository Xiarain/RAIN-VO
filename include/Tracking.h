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
#include "Initializer.h"

using namespace std;

namespace RAIN_VIO
{

class Camera;
class Frame;
class Map;
class GlobalSFM;
class Initializer;


enum eTrackingState
{
    NO_INITIALIZED = 0,
    OK = 1,
    LOST = 2,
    BAD = 3
};

enum eMarginFlag
{
    MARGINOLD = 0,
    MARGINSECONDNEW = 1
};

class Tracking
{

public:

    Camera *mpCamera;
    Map *mpMap;
    Initializer *mpInitializer;
    Feature *mpFeature;

    map<size_t, Frame *> mmpFrames;
    map<size_t, Frame *> mmpFramesWin;

    eTrackingState etrackingState;
    eMarginFlag eMarginflag;

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

    size_t mdFrameCount;

    int mnWindowSize;

    Tracking(const string &strSettingsFile, int nWindowSize);
    Tracking();
    ~Tracking();
    void Track(const cv::Mat &image, const double &TimeStamps);
    void SlideWindow();
    bool InitialStructure();

private:

    bool mbFirstImage;
    double mFirstImageTime;
    int mnumFeatures;
    int minDist;
    uint mIDcnt;
    string mstrSettingsFile;


}; // class Tracking

} // namespce RAIN_VIO

#endif //RAIN_VIO_TRACKING_H
