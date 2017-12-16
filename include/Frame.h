//
// Created by rain on 17-9-17.
//

#ifndef RAIN_VIO_FRAME_H
#define RAIN_VIO_FRAME_H

#include "Feature.h"
#include "Map.h"
#include <iostream>
#include <vector>

using namespace std;

namespace RAIN_VIO
{

class Feature;
class Camera;
class FeaturePerFrame;

class Frame
{

public:

    Eigen::Matrix<double, 3, 4> mTwc;
    Eigen::Matrix<double, 3, 4> mTcw;

    int mnWindowSize;
    cv::Mat mImageShow;
    cv::Mat mViwerShow;
    vector<pair<uint, Eigen::Vector3d> > mvFraFeatures; // ID and point vector<pair<int, Eigen::Vector3d>>
    vector<FeaturePerFrame> mvFeaturePerFrame;
    vector<cv::Point2f> mvFraPointsPts;
    vector<uint> mvFraPointsID;
    vector<int> mvFraPointsCnt;

    Frame();
    Frame(Camera *pCamera, Feature *pfeature, const string &strSettingsFile, const int nWindowSize);

    ~Frame();

    size_t GetFrameID();

    void DetectKeyPoint(const cv::Mat &image, const double &TimeStamps);

    void SetPose(Eigen::Matrix<double, 3, 4> Tcw);

    void SetPoseInverse(Eigen::Matrix<double, 3, 4> Twc);

    void SetPose(Eigen::Quaterniond Rqcw, Eigen::Vector3d tcw);

    Eigen::Matrix<double, 3, 4> GetPose();

    Eigen::Matrix<double, 3, 4> GetPoseInverse();

    void UpdatePoseMatrices();

    Eigen::Matrix3d GetRotation();

    Eigen::Vector3d GetTranslation();

    Eigen::Matrix3d GetRotationInverse();

    Eigen::Vector3d GetTranslationInverse();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

    static size_t gCount;

    size_t mID;

    Feature *mpFeature;
    Camera *mpCamera;

    Eigen::Matrix3d mRwc;
    Eigen::Vector3d mtwc;
    Eigen::Matrix3d mRcw;
    Eigen::Vector3d mtcw;
    Eigen::Quaterniond mRqcw;

    double ImageHeight;
    double ImageWidth;

    std::mutex mMutexPose;

};

}



#endif //RAIN_VIO_FRAME_H
