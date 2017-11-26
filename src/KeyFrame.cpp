//
// Created by rain on 17-9-19.
//

#include "KeyFrame.h"

namespace RAIN_VIO
{

KeyFrame::KeyFrame(Frame *F, Map *pMap): mID(F->GetFrameID()), mvFraPointsPts(F->mvFraPointsPts), mvFraPointsID(F->mvFraPointsID),
                                         mvFraPointsCnt(F->mvFraPointsCnt), mTcw(F->mTcw), mTwc(F->mTwc)

{

}

Eigen::Matrix<double, 3, 4> KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc;
}

Eigen::Matrix<double, 3, 4> KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw;
}


} // namesapce RAIN_VIO