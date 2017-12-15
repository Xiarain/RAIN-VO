//
// Created by rain on 17-11-27.
//

#include "string"
#include "iostream"

#include <eigen3/Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Camera.h>

#include "Converter.h"


using namespace std;

void FindFeatureMatches(const cv::Mat& imag1, const cv::Mat& imag2,
                        vector<cv::KeyPoint>& vKeyPoints1, vector<cv::KeyPoint>& vKeyPoints2,
                        vector<cv::DMatch>& vmatches);

cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& K);

void Triangulation(const vector<cv::KeyPoint> &vkeypoint1, const vector<cv::KeyPoint> &vkeypoint2,
                   const vector<cv::DMatch> &vmatches, const cv::Mat &R, const cv::Mat &t,
                   cv::Mat K, vector<cv::Point3d> &vpoints);

void RefineMatchesHomograpy(const vector<cv::KeyPoint> &vkeypoint1, const vector<cv::KeyPoint> &vkeypoint2,
                            vector<cv::DMatch>& vmatches, const double reprojectionThreshold,
                            const cv::Mat K, cv::Mat& homography);


int main(int argc, char** argv)
{
    const string strimg1FilePath = "../test/test_pnp/1.png";
    const string strimg2FilePath = "../test/test_pnp/2.png";
    const string strimg1depthFilePath = "../test/test_pnp/1_depth.png";
    const string strimg2depthFilePath = "../test/test_pnp/2_depth.png";

    cv::Mat img1 = cv::imread(strimg1FilePath, CV_LOAD_IMAGE_COLOR);
    if (img1.empty())
    {
        cout << "can not load the image " << strimg1FilePath << endl;
        return 0;
    }
    cv::Mat img2 = cv::imread(strimg2FilePath, CV_LOAD_IMAGE_COLOR);
    cv::Mat img1depth = cv::imread(strimg1depthFilePath, CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat img2depth = cv::imread(strimg2depthFilePath, CV_LOAD_IMAGE_UNCHANGED);

    vector<cv::KeyPoint> vkeypoints1, vkeypoints2;
    vector<cv::DMatch> vmatches;
    FindFeatureMatches(img1, img2, vkeypoints1, vkeypoints2, vmatches);

    cv::Mat K;
    K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    RAIN_VIO::Camera camera;
    camera.SetCameraIntrinsic(RAIN_VIO::Converter::toMatrix3d(K));

    cv::Mat homograpy;
//    RefineMatchesHomograpy(vkeypoints1, vkeypoints2, vmatches, 3., K, homograpy);

    cout << "the matches between the two images " << vmatches.size() << endl;

    cv::Mat showImg;
    cv::drawMatches(img1, vkeypoints1, img2, vkeypoints2, vmatches, showImg);

    vector<cv::Point3d> vpts3d;
    vector<cv::Point2d> vpts2d;

    for (auto m:vmatches)
    {
        // (y)[x] y*image.cols+x
        // queryIdx is the former descriptors
        // trainIdx is the later desciptors
        ushort d = img1depth.ptr<unsigned short> (int (vkeypoints1[m.queryIdx].pt.y)) [int(vkeypoints1[m.queryIdx].pt.x)];
        if (d == 0)
            continue;

        double dd = d/1000.0;
        cv::Point2d p1 = pixel2cam(vkeypoints1[m.queryIdx].pt, K);
        vpts3d.emplace_back(cv::Point3d(p1.x*dd, p1.y*dd, dd));
        vpts2d.emplace_back(vkeypoints2[m.trainIdx].pt);
    }

    cout << "3d-2d pairs: " << vpts3d.size() << endl;

    cv::Mat r, t;
    // the depth information comes from the first image
    cv::solvePnP(vpts3d, vpts2d, K, cv::Mat(), r, t, false);

    // from the camera to the world, also means from the frame 2 to the frame 1
    cv::Mat R;
    cv::Rodrigues(r, R);

    cout << "R " << R << endl;
    cout << "t " << t << endl;

    vector<cv::Point3d> vpoints;
    vector<Eigen::Vector3d> point3dw1;
    vector<Eigen::Vector3d> point3dw2;
    vector<cv::DMatch> vtrianmatches;

    {
        Eigen::Matrix3d R12;
        Eigen::Vector3d t12;
        Eigen::Vector3d Pointc;

        // from the frame 2 to the fram 1
        R12 = RAIN_VIO::Converter::toMatrix3d(R);
        t12 = RAIN_VIO::Converter::toVector3d(t);

        for (auto m:vmatches)
        {
            ushort d1 = img1depth.ptr<unsigned short> (int (vkeypoints1[m.queryIdx].pt.y)) [int(vkeypoints1[m.queryIdx].pt.x)];
            if (d1 == 0)
                continue;

            ushort d2 = img2depth.ptr<unsigned short> (int (vkeypoints2[m.trainIdx].pt.y)) [ int(vkeypoints2[m.trainIdx].pt.x)];
            if (d2 == 0)
                continue;

            double dd1 = d1/1000.0;
            point3dw1.emplace_back(camera.Pixwl2World(RAIN_VIO::Converter::toVector2d(vkeypoints1[m.queryIdx].pt),
                                                      Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0), dd1));
            double dd2 = d2/1000.0;
            point3dw2.emplace_back(camera.Pixwl2World(RAIN_VIO::Converter::toVector2d(vkeypoints2[m.trainIdx].pt), Eigen::Quaterniond(R12), t12, dd2));

            vtrianmatches.emplace_back(m);
        }

    }

    Triangulation(vkeypoints1, vkeypoints2, vtrianmatches, R, t, K, vpoints);

    for (int i = 0; i < point3dw2.size(); i++)
    {
        cout << "~" << point3dw1[i].transpose() << endl;
        cout << " " << point3dw2[i].transpose() << endl;
        cout << " " << vpoints[i] << endl;
    }

    cv::imshow(" ", showImg);
    cv::waitKey(0);

    return 0;
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

    cout << "max dist " << maxdist << endl;
    cout << "min dist " << mindist << endl;

    for (int i = 0; i < descriptors1.rows; i++)
    {
        if (vmatch[i].distance <= max(2*mindist, 30.0))
        {
            vmatches.push_back(vmatch[i]);
        }
    }

//    cv::Mat imggoodmatch;
//    cv::drawMatches ( imag1, vKeyPoints1, imag2, vKeyPoints2, vmatches, imggoodmatch );
//    cv::imshow ( "good match", imggoodmatch );
}

cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& K)
{
    cv::Point2d pts2d;

    pts2d.x = (p.x - K.at<double>(0, 2))/K.at<double>(0, 0);
    pts2d.y = (p.y - K.at<double>(1, 2))/K.at<double>(1, 1);

    return pts2d;
}

void Triangulation(const vector<cv::KeyPoint> &vkeypoint1, const vector<cv::KeyPoint> &vkeypoint2,
                   const vector<cv::DMatch> &vmatches, const cv::Mat &R, const cv::Mat &t,
                   cv::Mat K, vector<cv::Point3d> &vpoints)
{
    vector<cv::Point2d> vpts0, vpts1;

    for (cv::DMatch match : vmatches)
    {
        vpts0.push_back(pixel2cam(vkeypoint1[match.queryIdx].pt, K));
        vpts1.push_back(pixel2cam(vkeypoint2[match.trainIdx].pt, K));
    }

    cv::Mat pts4d;

#if 1

    // from the frame 1 to the frame 0
    Eigen::Matrix<double, 3, 4> T00;
    T00.block<3, 3>(0, 0).setIdentity();
    T00.block<3, 1>(0, 3).setZero();

    Eigen::Matrix<double, 3, 4> T01;
    T01 << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
           R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
           R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0);

    for (size_t i = 0; i < vpts0.size(); i++)
    {
        Eigen::Matrix<double, 4, 4> SVDA;

        SVDA.row(0) = vpts0[i].x*T00.row(2) - T00.row(0);
        SVDA.row(1) = vpts0[i].y*T00.row(2) - T00.row(1);
        SVDA.row(2) = vpts1[i].x*T01.row(2) - T01.row(0);
        SVDA.row(3) = vpts1[i].y*T01.row(2) - T01.row(1);

        Eigen::Vector4d SVDV = Eigen::JacobiSVD<Eigen::MatrixXd>(SVDA, Eigen::ComputeThinV).matrixV().rightCols<1>();

        SVDV = SVDV/SVDV[3];

        vpoints.emplace_back(cv::Point3d(SVDV[0], SVDV[1], SVDV[2]));
    }

#elif 0

    cv::Mat T0;
    T0 = (cv::Mat_<double> (3, 4) <<  1., 0., 0., 0.,
            0., 1., 0., 0.,
            0., 0., 1., 0.,
            0., 0., 0., 1.);

    // it mean from the frame 1 to frame 0
    cv::Mat T1;
    T1 = (cv::Mat_<double> (3,4) << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0));


    for (size_t i = 0; i < vpts0.size(); i++)
    {


        cv::Mat A(4,4,CV_64F);
        A.row(0) = vpts0[i].x*T0.row(2)-T0.row(0);
        A.row(1) = vpts0[i].y*T0.row(2)-T0.row(1);
        A.row(2) = vpts1[i].x*T1.row(2)-T1.row(0);
        A.row(3) = vpts1[i].y*T1.row(2)-T1.row(1);

        //A = WUV_t X是V的最后一列
        cv::Mat w,u,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

        // 三角化恢复的3D点
        cv::Mat x3D;
        x3D = vt.row(3).t();

        if(x3D.at<double>(3)==0)
            continue;

        // Euclidean coordinates
        x3D = x3D.rowRange(0,3)/x3D.at<double>(3);

        vpoints.emplace_back(cv::Point3d(x3D.at<double>(0), x3D.at<double>(1), x3D.at<double>(2)));
    }

#else

    cv::Mat T0;
    T0 = (cv::Mat_<double> (3, 4) <<  1., 0., 0., 0.,
                                      0., 1., 0., 0.,
                                      0., 0., 1., 0.,
                                      0., 0., 0., 1.);

    // it mean from the frame 1 to frame 0
    cv::Mat T1;
    T1 = (cv::Mat_<float> (3,4) << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
                                   R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
                                   R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0));

    cv::triangulatePoints(T0, T1, vpts0, vpts1, pts4d);

    for (int i = 0; i < pts4d.cols; i++)
    {
        cv::Mat x = pts4d.col(i);
        x /= x.at<double>(3, 0);
        cv::Point3d p(x.at<double>(0, 0), x.at<double>(1, 0), x.at<double>(2, 0));

        vpoints.push_back(p);
    }

#endif


}

void RefineMatchesHomograpy( const vector<cv::KeyPoint> &vkeypoint1, const vector<cv::KeyPoint> &vkeypoint2,
                             vector<cv::DMatch>& vmatches, const double reprojectionThreshold,
                             const cv::Mat K, cv::Mat& homography)
{
    if (vmatches.size() < 10)
        return;

    vector<cv::Point2f> vpts0;
    vector<cv::Point2f> vpts1;

    for (cv::DMatch match : vmatches)
    {
//        vpts0.push_back(pixel2cam(vkeypoint1[match.queryIdx].pt, K));
//        vpts1.push_back(pixel2cam(vkeypoint2[match.trainIdx].pt, K));

        vpts0.push_back(vkeypoint1[match.queryIdx].pt);
        vpts1.push_back(vkeypoint2[match.trainIdx].pt);
    }

    vector<uchar> inlierMask(vmatches.size());
    homography = cv::findHomography(vpts0, vpts1, CV_FM_RANSAC, reprojectionThreshold, inlierMask);

    vector<cv::DMatch> vinliersMatches;
    for (size_t i = 0; i < inlierMask.size(); i++)
    {
        if (inlierMask[i])
            vinliersMatches.push_back(vmatches[i]);
    }

    vmatches.swap(vinliersMatches);
}