//
// Created by rain on 17-9-17.
//

#ifndef RAIN_VIO_FRAME_H
#define RAIN_VIO_FRAME_H

#include "Tracking.h"
#include "Feature.h"
#include <iostream>
#include <vector>

using namespace std;

namespace RAIN_VIO
{

class Feature;
class Camera;

class Frame
{

public:

    Eigen::Matrix<double, 3, 4> mTcw;
    Eigen::Matrix<double, 3, 4> mTwc;

    int mnWindowSize;
    cv::Mat mImageShow;
    vector<pair<uint, Eigen::Vector3d> > mvFraFeatures; // ID and point vector<pair<int, Eigen::Vector3d>>
    vector<cv::Point2f> mvFraPointsPts;
    vector<uint> mvFraPointsID;
    vector<int> mvFraPointsCnt;

    Frame(Camera *pCamera, Feature *pfeature, const string &strSettingsFile, const int nWindowSize);

    ~Frame();

    void DetectKeyPoint(const cv::Mat &image, const double &TimeStamps);

    void SetPose(Eigen::Matrix<double, 3, 4> Tcw);

    void SetPoseInverse(Eigen::Matrix<double, 3, 4> Twc);

    Eigen::Matrix<double, 3, 4> GetPose();

    void UpdatePoseMatrices();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

    Feature *mpFeature;
    Camera *mpCamera;

    Eigen::Matrix3d mRcw;
    Eigen::Vector3d mtcw;
    Eigen::Matrix3d mRwc;
    Eigen::Vector3d mtwc;

    double ImageHeight;
    double ImageWidth;

};

}



#endif //RAIN_VIO_FRAME_H
