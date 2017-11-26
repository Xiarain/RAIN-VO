//
// Created by rain on 17-9-19.
//

#ifndef RAIN_VIO_KEYFRAME_H
#define RAIN_VIO_KEYFRAME_H


#include "Frame.h"
#include "Map.h"

namespace RAIN_VIO
{

class Frame;
class Map;

class KeyFrame
{
public:
    KeyFrame(Frame *F, Map *pMap);

    void SetPose(Eigen::Matrix<double, 3, 4> Tcw);

    Eigen::Matrix<double, 3, 4> mTcw;
    Eigen::Matrix<double, 3, 4> mTwc;

    vector<pair<uint, Eigen::Vector3d> > mvFraFeatures; // ID and point vector<pair<int, Eigen::Vector3d>>
    vector<cv::Point2f> mvFraPointsPts;
    vector<uint> mvFraPointsID;
    vector<int> mvFraPointsCnt;

    Eigen::Matrix<double, 3, 4> GetPose();

    Eigen::Matrix<double, 3, 4> GetPoseInverse();

private:

    size_t mID;

    mutex mMutexPose;
};

}



#endif //RAIN_VIO_KEYFRAME_H
