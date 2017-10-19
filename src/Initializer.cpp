//
// Created by rain on 17-10-7.
//

#include <opencv2/core/types.hpp>
#include <opencv/cv.hpp>
#include "Initializer.h"



namespace RAIN_VIO
{

Initializer::Initializer(cv::Mat _CmaeraK, Map *_Map, int nWindowSize) :mpMap(_Map)
{
    mCameraK = _CmaeraK;
    mnWindowSize = nWindowSize;
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

        if (inliercnt < 30)
            return false;

        cout << "in the fundamental matrix, the inlier points of the all points " << inliercnt << "of" << static_cast<int>(Corres.size()) << endl;

        return true;
    } // (Corres.size() > 15)
    else
    {
        cout << "the number of the correspendences is less than 15" << endl;
        return false;
    }

}

/**
 * @brief
 *        Notice: the number of the frames should be greater than the window size, and this function can work
 * @param RelativeR
 * @param RelativeT
 * @param idx
 * @return
 */
bool Initializer::RelativePose(Eigen::Matrix3d &RelativeR, Eigen::Vector3d &RelativeT, int &idx)
{

    for (int i = 0; i < mnWindowSize; i++)
    {
        vector<pair<Eigen::Vector3d, Eigen::Vector3d>> corres;

        corres = mpMap->GetCorresponding(i, mnWindowSize);

        cout << static_cast<int>(corres.size()) << endl;

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
        else
        {
            cout << "Warning: the number of the corresponding point between the two image is too littel:"
                 << static_cast<int>(corres.size()) << endl;
            continue;
        }
    }

    cout << "Warning:  solution of the F matrix in the initializtion can't be got " << endl;
    return false;
}


void GlobalSFM::TriangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                                 Eigen::Vector2d &Point0, Eigen::Vector2d &Point1, Eigen::Vector3d &Point3d)
{
    // x × （PX） = 0
    // AX = 0
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

    A.row(0) = Point0[0]*Pose0.row(2) - Pose0.row(0);
    A.row(1) = Point0[1]*Pose0.row(2) - Pose0.row(1);
    A.row(2) = Point1[0]*Pose1.row(2) - Pose1.row(0);
    A.row(3) = Point1[1]*Pose1.row(2) - Pose1.row(1);

    // SVD
    Eigen::Vector4d X;
    X = A.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();

    Point3d(0) = X(0)/X(3);
    Point3d(1) = X(1)/X(3);
    Point3d(2) = X(2)/X(3);
}

bool GlobalSFM::SolveFrameByPnP(Eigen::Matrix3d &RInitial, Eigen::Vector3d &PInitial, int i,
                                vector<SFMFeature> &vSFMFeature)
{
    vector<cv::Point2f> vPoint2D;
    vector<cv::Point3f> vPoint3D;

    for (int i = 0; i < mnFeatureNum; i++)
    {
        if (vSFMFeature[i].State != true)
            continue;
        Eigen::Vector2d Point2D;
    }

}


} // namespce RAIN_VIO
