//
// Created by rain on 17-9-17.
//

#include "Frame.h"

namespace RAIN_VIO
{

Frame::Frame(const string &strSettingsFile)
{
    mpfeature = new Feature(strSettingsFile);

}

Frame::~Frame()
{
    delete mpfeature;
}

void Frame::DetectKeyPoint(const cv::Mat &image, const double &TimeStamps)
{
    mImageShow = image.clone();

    mpfeature->ProcessImage(image.clone(), TimeStamps);

    mvFraPointsPts = mpfeature->UndistoredPoints(); // mvCurPointsPts
    mvFraPointsID = mpfeature->mvPointTrackID;

//    cout << "mvFraPointsID " << endl;
//    for (auto id : mvFraPointsID)
//    {
//        cout << id << endl;
//    }

    mvFraFeatures.clear();
    for (int i = 0; i < mvFraPointsPts.size(); i++) // vector<pair<uint, Eigen::Vector3d> >
    {
        double x = mvFraPointsPts[i].x;
        double y = mvFraPointsPts[i].y;
        double z = 1.0;
        uint id = mvFraPointsID[i];
        mvFraFeatures.emplace_back(make_pair(id, Eigen::Vector3d(x, y, z)));
    }

//    for (auto id : mvFraPointsID)
//        cout << id << endl;

//    mImageShow = mpfeature->UndistoredImage(image);

//    cv::cvtColor(mImageShow, mImageShow, CV_GRAY2RGB);


//    if (!mImageShow.empty())
//    {
//        for (int i = 0; i < mvFraPointsPts.size(); i++)
//        {
//            cv::circle(mImageShow, mvFraPointsPts[i], 2, cv::Scalar(255, 0, 0), 2);
//        }
//
//        cv::imshow("", mImageShow);
//        cv::waitKey(0);
//    }

//    cout << "the current frame: " << mvFraPointsPts.size() << endl;

}


} // namespace RAIN_VIO