//
// Created by rain on 17-9-24.
//

#include "Camera.h"


namespace RAIN_VIO
{

Camera::Camera(const string &strSettingsFile)
{
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at " << strSettingsFile << endl;
        exit(-1);
    }

    mfx = fsSettings["Camera.fx"];
    mfy = fsSettings["Camera.fy"];
    mcx = fsSettings["Camera.cx"];
    mcy = fsSettings["Camera.cy"];

    mk1 = fsSettings["Camera.k1"];
    mk2 = fsSettings["Camera.k2"];
    mp1 = fsSettings["Camera.p1"];
    mp2 = fsSettings["Camera.p2"];

    mImageHeight = fsSettings["Camera.height"];
    mImageWidth = fsSettings["Camera.width"];

}

/**
 * @brief dist
 * @param p the point of the normalized coordinate
 * @param pCorrected the point have been corrected
 */
void Camera::Distortion(const Eigen::Vector2d & p, Eigen::Vector2d & pCorrected)
{
    double x2, y2, xy, r2, rad_dist;

    x2 = p[0] * p[0];
    y2 = p[1] * p[1];
    xy = p[0] * p[1];
    r2 = x2 + y2;

    // NOTICE: there is no 1, because this calculation is the corrected number, not the corrected point.
    rad_dist = mk1*r2 + mk2*r2*r2;

    pCorrected[0] = p[0]*rad_dist + 2.0*mp1*xy + mp2*(r2 + 2.0*x2);
    pCorrected[1] = p[1]*rad_dist + 2.0*mp2*xy + mp1*(r2 + 2.0*y2);
}



}// namespace RAIN_VIO