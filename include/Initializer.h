//
// Created by rain on 17-10-7.
//

#ifndef RAIN_VIO_INITIALIZER_H
#define RAIN_VIO_INITIALIZER_H

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "Map.h"

using namespace std;

class Map;

namespace RAIN_VIO
{

class Initializer
{
public:

    Map *mpMap;

    Initializer(cv::Mat _CmaeraK, Map *_Map); //
    bool SolveRelativeRT(const vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &Corres, Eigen::Matrix3d &Rotation, Eigen::Vector3d &Translation);
    bool RelativePose(Eigen::Matrix3d &RelativeR, Eigen::Vector3d &RelativeT, int &idx);


private:
    cv::Mat mCameraK;


}; // class Initializer

} // namespace RAIN_VIO

#endif //RAIN_VIO_INITIALIZER_H
