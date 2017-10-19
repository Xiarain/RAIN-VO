//
// Created by rain on 17-10-7.
//

#ifndef RAIN_VIO_MAP_H
#define RAIN_VIO_MAP_H

#include <iostream>
#include <vector>
#include <list>
#include <eigen3/Eigen/Dense>

using namespace std;

namespace RAIN_VIO
{

// 2D feature point in the per frame
class FeaturePerFrame
{

public:
    FeaturePerFrame(const Eigen::Vector3d &_point)
    {
        z = _point(2);
        Point = _point/z;
    }
    Eigen::Vector3d Point;
    double z;
    double Parallax;
    Eigen::MatrixXd A;
    Eigen::MatrixXd b;

}; // class FeaturePerFrame

// the 3D feature point in the system, every point is difinizied.
class MapPoint
{
public:
    int mnFeatureID;
    int mnStartFrame;
    vector<FeaturePerFrame> mvFeaturePerFrame;
    int mnUsedNum;
    bool bisOutlier;
    double mdEstimatedDepth;
    int mnFlag;

    Eigen::Vector3d mdGtp;

    MapPoint(int _FeatureID, int _StartFrame)
            : mnFeatureID(_FeatureID), mnStartFrame(_StartFrame),
              mnUsedNum(0), mdEstimatedDepth(-1.0), mnFlag(0)
    {

    }

    int EndFrame();

}; // class MapPoint


class Map
{
public:

    int mnWindowSize;

    Map(int nWindowSize);
    int mLastTrackNum;
    list<MapPoint> mlMapPoints;

    bool AddFeatureCheckParallax(const int FrameCount, const vector<pair<uint, Eigen::Vector3d>> & Features);
    double ComputeParallax(const MapPoint &mapPoint, int FrameCount);
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> GetCorresponding(int FrameCount1, int FrameCount2);
    void RemoveBack();
    void RemoveFront(int FrameCount);
    void DebugShow();

}; // class Map


} // namespace RAIN_VIO
#endif //RAIN_VIO_MAP_H
