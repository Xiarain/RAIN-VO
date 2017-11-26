//
// Created by rain on 17-9-19.
//

#ifndef RAIN_VIO_TRACKING_H
#define RAIN_VIO_TRACKING_H

#include <string>
#include <iostream>
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "System.h"
#include "Camera.h"
#include "Frame.h"
#include "Feature.h"
#include "Map.h"
#include "Initializer.h"
#include "Parameters.h"
#include "Optimizer.h"

using namespace std;

namespace RAIN_VIO
{

class Camera;
class Frame;
class Map;
class GlobalSFM;
class Initializer;
class Viewer;
class MapDrawer;


enum eTrackingState
{
    NOINITIALIZED = 0,
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
    Viewer *mpViewer;
    MapDrawer *mpMapDrawer;

    map<size_t, Frame *> mmpFrames;

    Frame *mpCurrentFrame;

    eTrackingState etrackingState;
    eMarginFlag eMarginflag;

    cv::Mat CmaeraK;
    cv::Mat DistCoef;
    cv::Mat mRawImage;
    double ImageHeight;
    double ImageWidth;

    size_t mnFrameCount;
    static const int mnWindowSize = 10;
    bool EQUALIZE;

    Tracking(const string &strSettingsFile, Map *pMap, MapDrawer *pMapDrawer);
    Tracking();
    ~Tracking();

    void Track(const cv::Mat &image, const double &TimeStamps);

    void SlideWindow();

    bool InitialStructure();

    bool TrackReferenceKeyFrame();

    void SetViewer(Viewer *pViewer);

private:

    array<Frame *, (gWindowSize+1)> maFramesWin;

    bool mbFirstImage;
    double mFirstImageTime;
    int mnumFeatures;
    int minDist;
    uint mIDcnt;
    string mstrSettingsFile;

    mutex mMutex;



}; // class Tracking

} // namespce RAIN_VIO

#endif //RAIN_VIO_TRACKING_H
