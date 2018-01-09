//
// Created by rain on 17-10-7.
//

#ifndef RAIN_VIO_MAP_H
#define RAIN_VIO_MAP_H

#include <iostream>
#include <vector>
#include <list>
#include <eigen3/Eigen/Dense>
#include <glog/logging.h>

#include "Frame.h"
#include "KeyFrame.h"
#include "Tracking.h"
#include "Parameters.h"

using namespace std;

namespace RAIN_VIO
{

class Frame;
class KeyFrame;
class MapPoint;
// 2D feature point in the per frame
class FeaturePerFrame
{

public:

    double z;
    double Parallax;
    Eigen::Vector3d Point;
    Eigen::MatrixXd A;
    Eigen::MatrixXd b;
    MapPoint *mpInvertMapPoint;

    FeaturePerFrame(const Eigen::Vector3d &_point) {z = _point(2); Point = _point/z;}

    void SetMapPoint(MapPoint *pMapPoint) { mpInvertMapPoint = pMapPoint;}

}; // class FeaturePerFrame

// the 3D feature point in the system, every point is difinizied.
class MapPoint
{
public:
    int mnFeatureID;
    int mnStartFrame;
    int mnUsedNum;
    bool bisOutlier;
    double mdEstimatedDepth;
    int mnFlag;

    Eigen::Vector3d mPoint3d;
    vector<FeaturePerFrame> mvFeaturePerFrame;

    MapPoint(int _FeatureID, int _StartFrame) : mnFeatureID(_FeatureID), mnStartFrame(_StartFrame),
                                                mnUsedNum(0), mdEstimatedDepth(-1.0), mnFlag(0) {}

    int EndFrame();

}; // class MapPoint


class Map
{
public:

    static const int mnWindowSize=10;

    int mLastTrackNum;
    list<MapPoint> mlMapPoints;
    vector<KeyFrame *> mvpKeyFrames;

    Map();

    inline void SetCamera(Camera *pCamera) { mpCamera = pCamera;}

    bool AddFeatureCheckParallax(Frame *pFrame, const int FrameCount, const vector<pair<uint, Eigen::Vector3d>> & Features);

    double ComputeParallax(const MapPoint &mapPoint, int FrameCount);

    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> GetCorresponding(int FrameCount1, int FrameCount2);

    void RemoveBack();

    void RemoveFront(int FrameCount);

    int GetMapPointCount();

    void DebugShow();

    void Triangulate(array<Frame *, gWindowSize+1> *paFramesWin);

    void Triangulate2(Frame *pCurrentFrame, array<Frame *, gWindowSize+1> *paFramesWin);

    vector<KeyFrame *> GetAllKeyFrames();

    vector<MapPoint *> GetAllMapPoints();

protected:

    Camera *mpCamera;

    mutex mMutexMap;

}; // class Map


} // namespace RAIN_VIO
#endif //RAIN_VIO_MAP_H
