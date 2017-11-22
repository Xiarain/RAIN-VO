//
// Created by rain on 17-11-17.
//

#ifndef RAIN_VIO_OPTIMIZER_H
#define RAIN_VIO_OPTIMIZER_H

#include <vector>

#include "Frame.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "Converter.h"
#include "Parameters.h"
#include "Map.h"

namespace RAIN_VIO
{

class Frame;
class Map;

struct ReprojectionError2{

    ReprojectionError2(double observedx, double observedy)
            : observedx(observedx), observedy(observedy) {}

    template <typename T>
    bool operator()(const T* const Rqcw, const T* const tcw, const T* const Point3D, T* resuduals) const
    {
        T Point[3];
        ceres::QuaternionRotatePoint(Rqcw, Point3D, Point);

        Point[0] += tcw[0];
        Point[1] += tcw[1];
        Point[2] += tcw[2];

        // in the ceres BA example, it should apply second and fourth order radial distortion
        // but in the usually, the feature point have been distortion in the previous step

        T xp = Point[0]/Point[2];
        T yp = Point[1]/Point[2];

        resuduals[0] = xp - T(observedx);
        resuduals[1] = yp - T(observedy);

        return true;
    }

    static ceres::CostFunction* Create(const double observedu, const double observedv)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError2, 2, 4, 3, 3>(new ReprojectionError2(observedu, observedv)));
    }

    double observedx;
    double observedy;

}; // struct ReprojectionError2

class Optimizer
{

public:
    void static PoseOptimization(int IdxWin, Frame *pFrame, Map *pMap);

    Eigen::Vector2d ReprojectionError(const ceres::Problem& problem, ceres::ResidualBlockId id);

    std::vector<double> GetReprojectionErrorNorms(const ceres::Problem& problem);

    void RemoveOutliers(ceres::Problem& problem, double threshold);

}; // class Optimizer

}
#endif //RAIN_VIO_OPTIMIZER_H
