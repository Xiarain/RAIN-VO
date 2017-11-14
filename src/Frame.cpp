//
// Created by rain on 17-9-17.
//

#include "Frame.h"

namespace RAIN_VIO
{

Frame::Frame(const string &strSettingsFile, const int nWindowSize)
{
    mnWindowSize = nWindowSize;

    mpfeature = new Feature(strSettingsFile, mnWindowSize);

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at " << strSettingsFile << endl;
        exit(-1);
    }

    ImageHeight = fsSettings["Camera.height"];
    ImageWidth = fsSettings["Camera.width"];

}

Frame::~Frame()
{
    delete mpfeature;
}

void Frame::DetectKeyPoint(const cv::Mat &image, const double &TimeStamps)
{
    mvFraPointsPts.clear();
    mvFraPointsID.clear();

    mImageShow = image.clone();

    mpfeature->ProcessImage(image.clone(), TimeStamps);

    mvFraPointsPts = mpfeature->UndistoredPoints(); // mvCurPointsPts

    mvFraPointsID = mpfeature->mvPointTrackID;

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

void Frame::SetPose(Eigen::Matrix<double, 3, 4> Tcw)
{
    mTcw = Tcw;
    UpdatePoseMatrices();
}

void Frame::SetPoseInverse(Eigen::Matrix<double, 3, 4> Twc)
{
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
    mRcw = mTcw.block<3, 3>(0, 0);
    mtcw = mTcw.block<3, 1>(0, 3);

    mRwc = mRcw.inverse();
    mtwc = -mRcw.inverse()*mtcw;

    mTwc.block<3, 3>(0, 0) = mRwc;
    mTwc.block<3, 1>(0, 3) = mtwc;
}


} // namespace RAIN_VIO