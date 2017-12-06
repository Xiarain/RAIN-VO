//
// Created by rain on 17-12-1.
//

#ifndef RAIN_VIO_POSE_LOCAL_PARAMETERIZATION_H
#define RAIN_VIO_POSE_LOCAL_PARAMETERIZATION_H

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "se3.h"

class PoseLocalParameterization;
class ReprojectionErrorSE3;

class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;

    virtual bool ComputeJacobian(const double *x, double *jacobian) const;

    virtual int GlobalSize() const { return 7; };

    virtual int LocalSize() const { return 6; };

}; // class PoseLocalParameterization


class ReprojectionErrorSE3 : public ceres::SizedCostFunction<2, 7, 3>
{

private:

    double observedx;
    double observedy;

public:

    double fx;
    double fy;
    double cx;
    double cy;

    ReprojectionErrorSE3(double fx_, double fy_, double cx_, double cy_, double observedx_, double observedy_):
                        fx(fx_), fy(fy_), cx(cx_), cy(cy_), observedx(observedx_), observedy(observedy_) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

//    void check(double **parameters);

}; // class ReprojectionErrorSE3


#endif //RAIN_VIO_POSE_LOCAL_PARAMETERIZATION_H
