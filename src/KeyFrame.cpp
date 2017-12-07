//
// Created by rain on 17-9-19.
//

#include "KeyFrame.h"

namespace RAIN_VIO
{

KeyFrame::KeyFrame(Frame *F, Map *pMap): mID(F->GetFrameID()), mvFraPointsPts(F->mvFraPointsPts), mvFraPointsID(F->mvFraPointsID),
                                         mvFraPointsCnt(F->mvFraPointsCnt), mTwc(F->mTwc), mTcw(F->mTcw)

{

}

/**
 * @brief
 * @return from the world to the camera
 */
Eigen::Matrix<double, 3, 4> KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw;
}

Eigen::Matrix<double, 3, 4> KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc;
}

void KeyFrame::SetPose(Eigen::Matrix<double, 3, 4> Tcw)
{
    unique_lock<mutex> lock(mMutexPose);
    mTcw = Tcw;
}

} // namesapce RAIN_VIO