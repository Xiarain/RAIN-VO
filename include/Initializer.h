//
// Created by rain on 17-10-7.
//

#ifndef RAIN_VIO_INITIALIZER_H
#define RAIN_VIO_INITIALIZER_H

#include <iostream>
#include <vector>
#include <map>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
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
    Eigen::Vector3d Position;
    double Depth;
};// struct SFMFeature


struct ReprojectionError
{
    ReprojectionError(double observedu, double observedv) : observedu(observedu), observedv(observedv){}

    template <typename T>
    bool operator()( const T* const Rcw, const T* const tcw, const T* const Point3D, T* resuduals) const
    {
        T Point2D[3];

        ceres::QuaternionRotatePoint(Rcw, Point3D, Point2D);
        Point2D[0] += tcw[0];
        Point2D[1] += tcw[1];
        Point2D[2] += tcw[2];

        // there is very important thing, the observed feature points have been normalized in the pinhole camera model
        T xp = Point2D[0]/Point2D[2];
        T yp = Point2D[1]/Point2D[2];

        resuduals[0] = xp - T(observedu);
        resuduals[1] = yp - T(observedv);

        return true;
    }


    static ceres::CostFunction* Create(const double observedu, const double observedv)
    {
        /**
         * @brief ReprojectionError: costfunction
         *        2: the number of residuals
         *        4: number of parameters in block 0
         *        3: number of parameters in block 1
         *        3: number of parameters in block 2
         *    To get an auto differentiated cost function, you must define a class with a templated operator()
         *    (a functor) that computes the cost function in terms of the template parameter T.
         */

        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3, 3>(new ReprojectionError(observedu, observedv)));
    }

    double observedu;
    double observedv;


};


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

    bool Construct(int FrameNum, Eigen::Quaterniond *Rqcw, Eigen::Vector3d *tcw, int l, const Eigen::Matrix3d RelativeR, const Eigen::Vector3d RealtiveT,
                    vector<SFMFeature> &vSFMFeature, map<int, Eigen::Vector3d> &SFMTrackedPoints);

private:
    bool SolveFrameByPnP(Eigen::Matrix3d &RInitial, Eigen::Vector3d &PInitial, int FrameCount, vector<SFMFeature> &vSFMFeature);
    void TriangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1, Eigen::Vector2d &Point0, Eigen::Vector2d &Point1,
                          Eigen::Vector3d &Point3d);
    void TriangulateTwoFrames(int Frame0Count, Eigen::Matrix<double, 3, 4> &Pose0, int Frame1Count, Eigen::Matrix<double, 3, 4> &Pose1,
                               vector<SFMFeature> &vSFMFeature);
    int mnFeatureNum; // number of the 3D feature

}; // class SFM

} // namespace RAIN_VIO

#endif //RAIN_VIO_INITIALIZER_H
