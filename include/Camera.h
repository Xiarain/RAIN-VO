//
// Created by rain on 17-9-24.
//

#ifndef RAIN_VIO_CAMERA_H
#define RAIN_VIO_CAMERA_H

#include <string>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;

namespace RAIN_VIO
{

class Camera
{

public:

    double mk1;
    double mk2;
    double mp1;
    double mp2;
    double mfx;
    double mfy;
    double mcx;
    double mcy;
    int mImageHeight;
    int mImageWidth;

    Camera(const string &strSettingsFile);
    void Distortion(const Eigen::Vector2d & p, Eigen::Vector2d & du);
    void LiftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P);



}; // class Camera

} // namespace RAIN_VIO
#endif //RAIN_VIO_CAMERA_H
