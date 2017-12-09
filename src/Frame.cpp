//
// Created by rain on 17-9-17.
//

#include "Frame.h"

namespace RAIN_VIO
{

size_t Frame::gCount = 0;


Frame::Frame(Camera *pCamera, Feature *pFeature, const string &strSettingsFile, const int nWindowSize)
{
    gCount++;

    mID = gCount;

    mnWindowSize = nWindowSize;

    mpFeature = pFeature;
    mpCamera = pCamera;

    ImageHeight = mpCamera->mImageHeight;
    ImageWidth = mpCamera->mImageWidth;
}

Frame::~Frame()
{
}

void Frame::DetectKeyPoint(const cv::Mat &image, const double &TimeStamps)
{
    mvFraPointsPts.clear();
    mvFraPointsID.clear();

#if 0
    mImageShow = image.clone();
#endif

    mpFeature->ProcessImage(image, TimeStamps);

    mvFraPointsPts = mpFeature->UndistoredPoints(); // mvCurPointsPts
    mvFraPointsID = mpFeature->mvPointTrackID;
    mvFraPointsCnt = mpFeature->mvPointTrackcnt;
    mViwerShow = mpFeature->mViwerShow;

    mvFraFeatures.clear();
    for (int i = 0; i < mvFraPointsPts.size(); i++)
    {
        double x = mvFraPointsPts[i].x;
        double y = mvFraPointsPts[i].y;
        double z = 1.0;
        mvFraFeatures.emplace_back(make_pair(mvFraPointsID[i], Eigen::Vector3d(x, y, z)));
    }

#if 0
    mImageShow = mpfeature->UndistoredImage(image);

    cv::cvtColor(mImageShow, mImageShow, CV_GRAY2RGB);

    const double FOCAL_LENGTH = 460.0;
    vector<cv::Point2f> FraPointsdis = mvFraPointsPts;
    for (auto &point : FraPointsdis)
    {
        point.x = FOCAL_LENGTH*point.x + ImageWidth/2.0;
        point.y = FOCAL_LENGTH*point.y + ImageHeight/2.0;
    }

    if (!mImageShow.empty())
    {
        for (int i = 0; i < FraPointsdis.size(); i++)
        {
            cv::circle(mImageShow, FraPointsdis[i], 2, cv::Scalar(255, 0, 0), 2);
        }

        cv::imshow("", mImageShow);
        cv::waitKey(0);
    }
#endif

//    cout << "the current frame: " << mvFraPointsPts.size() << endl;

}

size_t Frame::GetFrameID()
{
    unique_lock<mutex> lock(mMutexPose);
    return mID;
}

/**
 * @brief
 * @param Twcx from the world to the camera
 */
void Frame::SetPose(Eigen::Matrix<double, 3, 4> Tcw)
{
    unique_lock<mutex> lock(mMutexPose);
    mTcw = Tcw;
    UpdatePoseMatrices();
}

/**
 * @brief
 * @param Tcwx from the camera to the world
 */
void Frame::SetPoseInverse(Eigen::Matrix<double, 3, 4> Twc)
{
    unique_lock<mutex> lock(mMutexPose);

    mTwc = Twc;

    mRwc = mTwc.block<3, 3>(0, 0);
    mtwc = mTwc.block<3, 1>(0, 3);

    mRcw = mRwc.inverse();
    mtcw = -mRwc.inverse()*mtwc;

    mTcw.block<3, 3>(0, 0) = mRcw;
    mTcw.block<3, 1>(0, 3) = mtcw;

}

/*
 * use the mTcw to update the other rotation and translation matrices
 */
void Frame::UpdatePoseMatrices()
{
    unique_lock<mutex> lock(mMutexPose);

    mRcw = mTcw.block<3, 3>(0, 0);
    mtcw = mTcw.block<3, 1>(0, 3);

    mRwc = mRcw.inverse();
    mtwc = -mRcw.inverse()*mtcw;

    mTwc.block<3, 3>(0, 0) = mRwc;
    mTwc.block<3, 1>(0, 3) = mtwc;
}

/**
 * @brief
 * @param Rqcw from the world to the camera
 * @param tcw
 */
void Frame::SetPose(Eigen::Quaterniond Rqcw, Eigen::Vector3d tcw)
{
    unique_lock<mutex> lock(mMutexPose);

    mRqcw = Rqcw;
    mtcw = tcw;
    mRcw = mRqcw.toRotationMatrix();

    mTcw.block<3, 3>(0, 0) = mRcw;
    mTcw.block<3, 1>(0, 3) = mtcw;

    mRwc = mRcw.inverse();
    mtwc = -mRcw.inverse()*mtcw;

    mTwc.block<3, 3>(0, 0) = mRwc;
    mTwc.block<3, 1>(0, 3) = mtwc;
}

/**
 * @brief
 * @return from the world to the camera
 */
Eigen::Matrix<double, 3, 4> Frame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw;
}

Eigen::Matrix<double, 3, 4> Frame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc;
}

Eigen::Matrix3d Frame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return mRcw;
}

Eigen::Vector3d Frame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return mtcw;
}

Eigen::Matrix3d Frame::GetRotationInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return mRwc;
}

Eigen::Vector3d Frame::GetTranslationInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return mtwc;
}

} // namespace RAIN_VIO