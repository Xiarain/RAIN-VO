//
// Created by rain on 17-10-7.
//

#include <opencv2/core/types.hpp>
#include <opencv/cv.hpp>
#include "Initializer.h"
#include "Converter.h"



namespace RAIN_VIO
{

Initializer::Initializer(cv::Mat _CmaeraK, Map *_Map, int nWindowSize) :mpMap(_Map)
{
    mCameraK = _CmaeraK;
    mnWindowSize = nWindowSize;
}

bool Initializer::SolveRelativeRT(const vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &Corres,
                                  Eigen::Matrix3d &Rotation, Eigen::Vector3d &Translation)
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

        LOG(INFO) << "The inlier points of the all points " << inliercnt << "of" << static_cast<int>(Corres.size()) << endl;

        return true;
    } // (Corres.size() > 15)
    else
    {
        LOG(WARNING) << "The number of the correspendences is less than 15" << endl;
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
            LOG(WARNING)  << "The number of the corresponding point between the two image is too littel:"
                 << static_cast<int>(corres.size()) << endl;
            continue;
        }
    }

    LOG(WARNING) << "Solution of the F matrix in the initializtion can't be got " << endl;
    return false;
}


void GlobalSFM::TriangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                                 Eigen::Vector2d &Point0, Eigen::Vector2d &Point1, Eigen::Vector3d &Point3d)
{
    // x × （PX） = 0
    // AX = 0
    // multi view geometry P 217
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

/**
 * @brief
 * @param RInitial Rcw
 * @param PInitial tcw
 * @param FrameCount
 * @param vSFMFeature
 * @return
 */
bool GlobalSFM::SolveFrameByPnP(Eigen::Matrix3d &RInitial, Eigen::Vector3d &PInitial, int FrameCount,
                                vector<SFMFeature> &vSFMFeature)
{
    // those 2D point and 3D point are required in the PnP problem
    vector<cv::Point2f> vPoint2D;
    vector<cv::Point3f> vPoint3D;

    for (int i = 0; i < mnFeatureNum; i++)
    {
        if (vSFMFeature[i].State != true)
            continue;

        for (size_t j = 0; j < vSFMFeature[i].Observation.size(); j++)
        {
            // if the 2D feature is observed in the frame FrameCount, add the 2D feature to the PnP
            if (vSFMFeature[i].Observation[j].first == FrameCount)
            {
                vPoint2D.emplace_back(Converter::toCvPoint2f(vSFMFeature[i].Observation[j].second));

                vPoint3D.emplace_back(Converter::toCvPoint3f(vSFMFeature[i].Position));

                break;
            }
        }
    }

    if (static_cast<int>(vPoint2D.size()) < 15)
    {
        LOG(WARNING) << "the matches is short in the PnP solution" <<endl;
        return false;
    }

    cv::Mat r, rvec, t, D, tmpr;

    cv::eigen2cv(RInitial, tmpr);

    // turn the rotation matrix to the rotation vector
    cv::Rodrigues(tmpr, rvec);

    cv::eigen2cv(PInitial, t);

    cv::Mat K;
    K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0,
                                   0, 1.0, 0,
                                   0, 0, 1.0);

    if (!cv::solvePnP(vPoint3D, vPoint2D, K, D, rvec, t, 1))
    {
        LOG(ERROR) << " failed in solving the PnP problem " << endl;
        return false;
    }

    cv::Rodrigues(rvec, r);

    Eigen::MatrixXd RPnP;
    cv::cv2eigen(r, RPnP);
    Eigen::MatrixXd TPnP;
    cv::cv2eigen(t,TPnP);

    RInitial = RPnP;
    PInitial = TPnP;

    return true;
}

void GlobalSFM::TriangulateTwoFrames(int Frame0Count, Eigen::Matrix<double, 3, 4> &Pose0, int Frame1Count,
                                     Eigen::Matrix<double, 3, 4> &Pose1, vector<SFMFeature> &vSFMFeature)
{
    if (Frame0Count == Frame1Count)
    {
        LOG(FATAL) << "the index of the two frames should not be same" << endl;
        return;
    }

    for (int i = 0; i < mnFeatureNum; i++)
    {
        if (vSFMFeature[i].State)
            continue;

        Eigen::Vector2d Point02d;
        Eigen::Vector2d Point12d;
        bool bFrame0 = false, bFrame1 = false;

        for (size_t j = 0; j < int(vSFMFeature[i].Observation.size()); j++)
        {
            if (vSFMFeature[i].Observation[j].first == Frame0Count)
            {
                Point02d = vSFMFeature[i].Observation[j].second;
                bFrame0 = true;
            }

            if (vSFMFeature[i].Observation[j].first == Frame1Count)
            {
                Point12d = vSFMFeature[i].Observation[j].second;
                bFrame1 = true;
            }
        }

        if (bFrame0 && bFrame1)
        {
            Eigen::Vector3d Point3d;

            TriangulatePoint(Pose0, Pose1, Point02d, Point12d, Point3d);

            vSFMFeature[i].State = true;
            vSFMFeature[i].Position = Point3d;
        }

    }

}

/**
 * @brief
 * @param FrameNum
 * @param Rqcw rotation by the quaternion
 * @param tcw
 * @param l the frame has the big parallax between the l frame and the newest frame in the slide window
 * @param RelativeR the relation of the two frames which have the big parallax
 * @param RealtiveT
 * @param vSFMFeature
 * @param SFMTrackedPoints
 * @return
 */
bool  GlobalSFM::Construct(int FrameNum, Eigen::Quaterniond *Rqcw, Eigen::Vector3d *tcw, int l,
                           const Eigen::Matrix3d RelativeR, const Eigen::Vector3d RealtiveT,
                           vector<SFMFeature> &vSFMFeature, map<int, Eigen::Vector3d> &SFMTrackedPoints)
{
    mnFeatureNum = (int)vSFMFeature.size();

    Rqcw[l].w() = 1;
    Rqcw[l].x() = 0;
    Rqcw[l].y() = 0;
    Rqcw[l].z() = 0;
    tcw[l].setZero();

    // the coordination of the newest frame
    Rqcw[FrameNum-1] = Rqcw[l]*Eigen::Quaterniond(RelativeR);
    tcw[FrameNum-1] = RealtiveT;

    Eigen::Matrix3d Rwc[FrameNum]; //RotationCam
    Eigen::Vector3d twc[FrameNum]; // TranslationCam
    Eigen::Quaterniond Rqwc[FrameNum]; // QuatCam

    Eigen::Matrix<double, 3, 4> Twc[FrameNum]; // Pose

    // just the transformation matrix's inverse
    Rqwc[l] = Rqcw[l].inverse();
    Rwc[l] = Rqwc[l].toRotationMatrix();
    twc[l] = -1*Rwc[l]*tcw[l];
    Twc[l].block<3, 3>(0, 0) = Rwc[l];
    Twc[l].block<3, 1>(0, 3) = twc[l];

    Rqwc[FrameNum-1] = Rqcw[FrameNum-1].inverse();
    Rwc[FrameNum-1] = Rqwc[FrameNum-1].toRotationMatrix();
    twc[FrameNum-1] = -1*Rwc[FrameNum-1]*tcw[FrameNum-1];
    Twc[FrameNum-1].block<3, 3>(0, 0) = Rwc[FrameNum-1];
    Twc[FrameNum-1].block<3, 1>(0, 3) = twc[FrameNum-1];

    for (int i = l; i < FrameNum - 1; i++)
    {
        if (i > l)
        {
            Eigen::Matrix3d RInitial = Rwc[i-1];
            Eigen::Vector3d PInitial = twc[i-1];

            if (!SolveFrameByPnP(RInitial, PInitial, i, vSFMFeature))
            {
                return false;
            }

            Rwc[i] = RInitial; // this is Rcw in the true meaning
            twc[i] = PInitial;
            Rqwc[i] = Rwc[i];

            Twc[i].block<3, 3>(0, 0) = Rwc[i];
            Twc[i].block<3, 1>(0, 3) = twc[i];
        }

        TriangulateTwoFrames(i, Twc[i], FrameNum-1, Twc[FrameNum-1], vSFMFeature);
    }

    for (int i = l+1; i < FrameNum-1; i++)
    {
        TriangulateTwoFrames(l, Twc[l], i, Twc[i], vSFMFeature);
    }

    for (int i = l-1; i >= 0; i--)
    {
        Eigen::Matrix3d RInitial = Rwc[i+1];
        Eigen::Vector3d PInitial = twc[i+1];

        if (!SolveFrameByPnP(RInitial, PInitial, i, vSFMFeature))
            return false;

        Rwc[i] = RInitial;
        twc[i] = PInitial;
        Rqwc[i] = Rwc[i];
        Twc[i].block<3, 3>(0, 0) = Rwc[i];
        Twc[i].block<3, 1>(0, 3) = twc[i];

        TriangulateTwoFrames(i, Twc[i], l, Twc[l], vSFMFeature);
    }

    for (int i = 0; i < mnFeatureNum; i++)
    {
        if (vSFMFeature[i].State)
            continue;

        // the 3D point with the several 2D point
        if ((int)vSFMFeature[i].Observation.size() >= 2)
        {
            Eigen::Vector2d Point2d0, Point2d1;

            int Frame0 = vSFMFeature[i].Observation[0].first;
            Point2d0 = vSFMFeature[i].Observation[0].second;

            int Frame1 = vSFMFeature[i].Observation.back().first;
            Point2d1 = vSFMFeature[i].Observation.back().second;

            Eigen::Vector3d Point3d;

            TriangulatePoint(Twc[Frame0], Twc[Frame1], Point2d0, Point2d1, Point3d);

            vSFMFeature[i].State = true;
            vSFMFeature[i].Position = Point3d;
        }
    }

    for (int i = 0; i < FrameNum; i++)
    {
        cout << "the camera pose in the SFM " << i << " frame" <<endl;
        cout << Converter::toEuler(Eigen::Quaterniond(Rwc[i])).transpose() << endl;
        cout << twc[i].transpose() << endl;
    }

    ceres::Problem problem;
    ceres::LocalParameterization* localparameterization = new ceres::QuaternionParameterization();

    double Rdwc[FrameNum][4];
    double tdwc[FrameNum][4];
    double Point3dw[mnFeatureNum][3];
//    Eigen::Matrix<double, 3, 4> Twc[FrameNum];

    // the camera parameter
    for (int i = 0; i < FrameNum; i++)
    {
        tdwc[i][0] = twc[i][0];
        tdwc[i][1] = twc[i][1];
        tdwc[i][2] = twc[i][2];

        Rdwc[i][0] = Rqwc[i].w();
        Rdwc[i][1] = Rqwc[i].x();
        Rdwc[i][2] = Rqwc[i].y();
        Rdwc[i][3] = Rqwc[i].z();

        problem.AddParameterBlock(Rdwc[i], 4, localparameterization);
        problem.AddParameterBlock(tdwc[i], 3);

        if (i == l)
        {
            problem.SetParameterBlockConstant(Rdwc[i]);
        }

        if (i == l || i == FrameNum-1)
        {
            problem.SetParameterBlockConstant(tdwc[i]);
        }
    }

    cout << "the 3D feature point" << endl;
    // the 3D feature point parameter
    for (int i = 0; i < mnFeatureNum; i++)
    {
        if (!vSFMFeature[i].State)
            continue;

        cout << i << " " << vSFMFeature[i].Position.transpose() <<endl;

        for (int j = 0; j < int(vSFMFeature[i].Observation.size()); j++)
        {
            int idxcamera = vSFMFeature[i].Observation[j].first;

            ceres::CostFunction* costFunction = ReprojectionError::Create(
                                                                   vSFMFeature[i].Observation[j].second[0],
                                                                   vSFMFeature[i].Observation[j].second[1]);
            Point3dw[i][0] = vSFMFeature[i].Position[0];
            Point3dw[i][1] = vSFMFeature[i].Position[1];
            Point3dw[i][2] = vSFMFeature[i].Position[2];

            problem.AddResidualBlock(costFunction, NULL, Rdwc[idxcamera], tdwc[idxcamera], Point3dw[i]);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true; // display the every iteration step
    options.max_solver_time_in_seconds = 0.35;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << endl;

    if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03)
    {
        cout << "the BA of SFM converge" << endl;
    }
    else
    {
        cout << "the BA of SFM can't converge" << endl;
        return false;
    }



}


} // namespce RAIN_VIO
