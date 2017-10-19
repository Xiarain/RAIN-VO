//
// Created by rain on 17-10-7.
//

#ifndef RAIN_VIO_INITIALIZER_H
#define RAIN_VIO_INITIALIZER_H

#include <iostream>
#include <vector>
#include <map>
#include <eigen3/Eigen/Dense>

#include "Map.h"

using namespace std;

class Map;
class SFM;

namespace RAIN_VIO
{

struct SFMFeature
{
    bool State;
    int Id;
    vector<pair<int , Eigen::Vector2d>> Observation;
    double Position[3];
    double Depth;
};// struct SFMFeature


class Initializer
{
public:

    Map *mpMap;
    int mnWindowSize;

    Initializer(cv::Mat _CmaeraK, Map *_Map, int nWindowSize); //
    bool SolveRelativeRT(const vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &Corres, Eigen::Matrix3d &Rotation, Eigen::Vector3d &Translation);
    bool RelativePose(Eigen::Matrix3d &RelativeR, Eigen::Vector3d &RelativeT, int &idx);

private:
    cv::Mat mCameraK;


}; // class Initializer

class GlobalSFM
{
public:
    GlobalSFM();
    bool Construct(int FrameNum, Eigen::Quaterniond &q, Eigen::Vector3d &T, int l, const Eigen::Matrix3d RelativeR, const Eigen::Vector3d RealtiveT,
                    vector<SFMFeature> &vSFMFeature, map<int, Eigen::Vector3d> &SFMTrackedPoints);

private:
    bool SolveFrameByPnP(Eigen::Matrix3d &RInitial, Eigen::Vector3d &PInitial, int i, vector<SFMFeature> &vSFMFeature);
    void TriangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1, Eigen::Vector2d &Point0, Eigen::Vector2d &Point1,
                          Eigen::Vector3d &Point3d);
    void TriangulateTWOFrames(int Frame0, Eigen::Matrix<double, 3, 4> &Pose0, int Frame1, Eigen::Matrix<double, 3, 4> &Pose1, vector<SFMFeature> &vSFMFeature);
    int mnFeatureNum;

}; // class SFM

} // namespace RAIN_VIO

#endif //RAIN_VIO_INITIALIZER_H
