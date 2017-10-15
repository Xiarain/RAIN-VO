//
// Created by rain on 17-10-7.
//

#include <opencv2/core/types.hpp>
#include <opencv/cv.hpp>
#include "Initializer.h"



namespace RAIN_VIO
{

Initializer::Initializer(cv::Mat _CmaeraK, Map *_Map) :mpMap(_Map)
{
    mCameraK = _CmaeraK;
}

bool Initializer::SolveRelativeRT(const vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &Corres, Eigen::Matrix3d &Rotation, Eigen::Vector3d &Translation)
{
    if (Corres.size() > 15)
    {
        vector<cv::Point2f> p1, p2;

        for (int i = 0; i < int(Corres.size()); i++)
        {
            p1.emplace_back(cv::Point2f(Corres[i].first(0), Corres[i].first(1)));
            p2.emplace_back(cv::Point2f(Corres[i].second(0), Corres[i].second(1)));
        }

//        cv::Mat mask;
        cv::Mat E = cv::findFundamentalMat(p1, p2, cv::FM_RANSAC, 0.3/460, 0.99);

        cv::Mat rot, trans;
        cv::Mat cameraMatrix = cv::Mat(3,3,CV_32F);
        cameraMatrix = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1.0);

        int inliercnt = cv::recoverPose(E, p1, p2, cameraMatrix, rot, trans);

        Eigen::Matrix3d R;
        Eigen::Vector3d T;

        for (int i = 0; i < 3; i++)
        {
            T(i) = trans.at<double>(i, 0);
            for (int j = 0; j < 3; j++)
                R(i, j) = rot.at<double>(i, j);
        }

        Rotation = R.transpose();
        Translation = -R.transpose()*T;

        if (inliercnt < 12)
            return false;

        return true;
    } // (Corres.size() > 15)
    else
    {
        cout << "the number of the correspendences is less than 15" << endl;
        return false;
    }

}

bool Initializer::RelativePose(Eigen::Matrix3d &RelativeR, Eigen::Vector3d &RelativeT, int &idx)
{
    const int WindowSize = 10;


    for (int i = 0; i < WindowSize; i++)
    {
        vector<pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
        corres = mpMap->GetCorresponding(i, WindowSize);

        if (corres.size() > 20)
        {
            double ParallaxSum = 0;
            double ParallaxAver = 0;

            for (int j = 0; j < int(corres.size()); j++)
            {
                Eigen::Vector2d pts1(corres[i].first(0), corres[i].first(1));
                Eigen::Vector2d pts2(corres[j].first(0), corres[j].first(1));

                double Parallax = (pts1 - pts2).norm();
                ParallaxSum = ParallaxSum + Parallax;
            }

            ParallaxAver = 1.0*ParallaxSum/int(corres.size());

            if (ParallaxAver*460 > 30 && SolveRelativeRT(corres, RelativeR, RelativeT))
            {
                idx = i;
                return true;
            }
        }
    }

    cout << " solution of the F matrix in the initializtion can't be got " << endl;
    return false;
}


} // namespce RAIN_VIO
