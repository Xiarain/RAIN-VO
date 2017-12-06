//
// Created by rain on 17-12-1.
//
#include <iostream>

#include <eigen3/Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "pose_local_parameterization.h"
#include "se3.h"

using namespace std;

cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& K);
void FindFeatureMatches(const cv::Mat& imag1, const cv::Mat& imag2,
                        vector<cv::KeyPoint>& vKeyPoints1, vector<cv::KeyPoint>& vKeyPoints2,
                        vector<cv::DMatch>& vmatches);
void PoseOptimization(vector<cv::Point3d> vPoints3d, vector<cv::Point2d> vPoints2d,
                      cv::Mat& K, cv::Mat& R, cv::Mat& t);

void ComputeReprojectionCost(const vector<cv::Point3d> vPoints3d, const vector<cv::Point2d> vPoints2d,
                             const cv::Mat& K, const cv::Mat& R, const cv::Mat& t);

int main(int argc, char* argv[])
{
    cout << "test the BA" << endl;

    {
        cout << "the SE3 class test " << endl;
        Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
        Eigen::Vector3d t(1,0,0);
        SE3 SE3Rt(R, t);

        Vector6d updatese3;
        updatese3.setZero();
        updatese3(0, 0) = 1e-4d;
        SE3 se3updated = SE3::exp(updatese3)*SE3Rt;

        cout << se3updated.log().transpose() << endl;
        cout << SE3Rt.log().transpose()  << endl;
        cout << se3updated << endl;
    }

    const string strimg1FilePath = "../test/test_pnp_material/1.png";
    const string strimg2FilePath = "../test/test_pnp_material/2.png";
    const string strimg1depthFilePath = "../test/test_pnp_material/1_depth.png";

    cv::Mat img1 = cv::imread(strimg1FilePath, CV_LOAD_IMAGE_COLOR);
    if (img1.empty())
    {
        cout << "can not load the image " << strimg1FilePath << endl;
    }
    cv::Mat img2 = cv::imread(strimg2FilePath, CV_LOAD_IMAGE_COLOR);
    cv::Mat img1depth = cv::imread(strimg1depthFilePath, CV_LOAD_IMAGE_UNCHANGED);

    vector<cv::KeyPoint> vkeypoints1, vkeypoints2;
    vector<cv::DMatch> vmatches;
    FindFeatureMatches(img1, img2, vkeypoints1, vkeypoints2, vmatches);

    cv::Mat showImg;
    cv::drawMatches(img2, vkeypoints2, img1, vkeypoints1, vmatches, showImg, CV_RGB(0, 255, 0), CV_RGB(0, 0, 255));

    cv::Mat K = (cv::Mat_<double> (3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    vector<cv::Point3d> vpts3d;
    vector<cv::Point2d> vpts2d;

    for (auto m:vmatches)
    {
        // (y)[x] y*image.cols+x
        // queryIdx is the former descriptors
        // trainIdx is the later desciptors
        ushort d = img1depth.ptr<unsigned short> (int ( vkeypoints1[m.queryIdx].pt.y )) [ int ( vkeypoints1[m.queryIdx].pt.x ) ];
        if (d == 0)
            continue;

        float dd = d/1000.0;
        cv::Point2d p1 = pixel2cam(vkeypoints1[m.queryIdx].pt, K);
        vpts3d.push_back(cv::Point3d(p1.x*dd, p1.y*dd, dd));
        vpts2d.push_back(vkeypoints2[m.trainIdx].pt);
    }

    cv::Mat r, t;
    cv::solvePnP(vpts3d, vpts2d, K, cv::Mat(), r, t, false);

    cv::Mat R;
    cv::Rodrigues(r, R);

    PoseOptimization(vpts3d, vpts2d, K, R, t);

    ComputeReprojectionCost(vpts3d, vpts2d, K, R, t);

    while (1)
    {
        cv::imshow(" ", showImg);
        cv::waitKey(10);
    }

    return 0;
}

cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& K)
{
    cv::Point2d pts2d;

    pts2d.x = (p.x - K.at<double>(0, 2))/K.at<double>(0, 0);
    pts2d.y = (p.y - K.at<double>(1, 2))/K.at<double>(1, 1);

    return pts2d;
}

void FindFeatureMatches(const cv::Mat& imag1, const cv::Mat& imag2,
                        vector<cv::KeyPoint>& vKeyPoints1, vector<cv::KeyPoint>& vKeyPoints2,
                        vector<cv::DMatch>& vmatches)
{
    cv::Mat descriptors1, descriptors2;

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );

    detector->detect(imag1, vKeyPoints1);
    detector->detect(imag2, vKeyPoints2);

    descriptor->compute(imag1, vKeyPoints1, descriptors1);
    descriptor->compute(imag2, vKeyPoints2, descriptors2);

    vector<cv::DMatch> vmatch;
    matcher->match(descriptors1, descriptors2, vmatch);

    double mindist=10000, maxdist=0;

    for (int i = 0; i < descriptors1.rows; i++)
    {
        double dist = vmatch[i].distance;
        if (dist < mindist) mindist = dist;
        if (dist > maxdist) maxdist = dist;
    }

    for (int i = 0; i < descriptors1.rows; i++)
    {
        if (vmatch[i].distance <= max(2*mindist, 30.0))
        {
            vmatches.push_back(vmatch[i]);
        }
    }
}

void PoseOptimization(vector<cv::Point3d> vPoints3d, vector<cv::Point2d> vPoints2d,
                      cv::Mat& K, cv::Mat& R, cv::Mat& t)
{
    cv::Mat intrinsic(cv::Matx41d(K.at<double>(0, 0), K.at<double>(1, 1), K.at<double>(0, 2), K.at<double>(1, 2)));

    cv::Mat extrinsic(7, 1, CV_64FC1);

    {
        Eigen::Matrix3d Reigen;
        cv::cv2eigen(R, Reigen);
        Eigen::Quaterniond Rq(Reigen);
        extrinsic.ptr<double>()[0] = Rq.x();
        extrinsic.ptr<double>()[1] = Rq.y();
        extrinsic.ptr<double>()[2] = Rq.z();
        extrinsic.ptr<double>()[3] = Rq.w();
        t.copyTo(extrinsic.rowRange(4, 7));
    }

    cout << "extrinsic: " << endl << extrinsic.t() << endl;

    ceres::Problem problem;

    problem.AddParameterBlock(extrinsic.ptr<double>(), 7, new PoseLocalParameterization());


    ceres::LossFunction* loss_function = new ceres::HuberLoss(4);   // loss function make bundle adjustment robuster.

    for (int i = 0; i < vPoints3d.size(); i++)
    {
        cv::Point2d observed = vPoints2d[i];

        ceres::CostFunction* costfunction = new ReprojectionErrorSE3(K.at<double>(0, 0), K.at<double>(1, 1),
                                                                     K.at<double>(0, 2), K.at<double>(1, 2),
                                                                     observed.x, observed.y);

        problem.AddResidualBlock(
                costfunction, nullptr,
                extrinsic.ptr<double>(), &vPoints3d[i].x);

        problem.AddParameterBlock(&vPoints3d[i].x, 3);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.3;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable())
    {
        cout << "Bundle Adjustment failed." << std::endl;
    }
    else
    {
        // Display statistics about the minimization
        cout << summary.BriefReport() << endl
             << " residuals number: " << summary.num_residuals << endl
             << " Initial RMSE: " << sqrt(summary.initial_cost / summary.num_residuals) << endl
             << " Final RMSE: " << sqrt(summary.final_cost / summary.num_residuals) << endl
             << " Time (s): " << summary.total_time_in_seconds << endl;

        cout << extrinsic.t() << endl;
    }
} // void PoseOptimization()

void ComputeReprojectionCost(const vector<cv::Point3d> vPoints3d, const vector<cv::Point2d> vPoints2d,
                             const cv::Mat& K, const cv::Mat& R, const cv::Mat& t)
{
    double cost=0;
    vector<Eigen::Vector2d> vresiduals;
    vresiduals.resize(vPoints3d.size());

    cv::Mat r;
    cv::Rodrigues(R, r);

    double Point3dw0[3];
    double Point3dw1[3];

    for (int i = 0; i < vPoints3d.size(); i++)
    {
        Point3dw0[0] = vPoints3d[i].x;
        Point3dw0[1] = vPoints3d[i].y;
        Point3dw0[2] = vPoints3d[i].z;

        ceres::AngleAxisRotatePoint(r.ptr<double>(), Point3dw0, Point3dw1);

        Point3dw1[0] += t.ptr<double>()[0];
        Point3dw1[1] += t.ptr<double>()[1];
        Point3dw1[2] += t.ptr<double>()[2];

//        double xp = K.at<double>(0, 0)*vPoints3d[i].x/vPoints3d[i].z + K.at<double>(0, 2);
//        double yp = K.at<double>(1, 1)*vPoints3d[i].y/vPoints3d[i].z + K.at<double>(1, 2);

        double xp = K.at<double>(0, 0)*Point3dw1[0]/Point3dw1[2] + K.at<double>(0, 2);
        double yp = K.at<double>(1, 1)*Point3dw1[1]/Point3dw1[2] + K.at<double>(1, 2);

        vresiduals[i][0] = xp - vPoints2d[i].x;
        vresiduals[i][1] = yp - vPoints2d[i].y;
    }

    for (auto residul:vresiduals)
    {
        cost += (residul[0]*residul[0] + residul[1]*residul[1]);
    }

    cost *= 0.5;

    cout << "Compute Reprojection Cost: " << cost << endl;
}

