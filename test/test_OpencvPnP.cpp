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
#include <ceres/ceres.h>
#include <ceres/rotation.h>


using namespace std;

void FindFeatureMatches(const cv::Mat& imag1, const cv::Mat& imag2,
                        vector<cv::KeyPoint>& vKeyPoints1, vector<cv::KeyPoint>& vKeyPoints2,
                        vector<cv::DMatch>& vmatches);

void PoseOptimization(const vector<cv::Point3f> vPoints3d, const vector<cv::Point2f> vPoints2d,
                      const cv::Mat& K, const cv::Mat& R, const cv::Mat& t);

void CalcuReprojectionCost(const vector<cv::Point3f> vPoints3d, const vector<cv::Point2f> vPoints2d,
                           const cv::Mat& K, const cv::Mat& R, const cv::Mat& t);

cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& K);



double gKd[4];


int main(int argc, char** argv)
{
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

    cout << "the matches between the two images " << vmatches.size() << endl;

    cv::Mat K = (cv::Mat_<double> (3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    gKd[0] = K.at<double>(0, 0); // fx
    gKd[1] = K.at<double>(1, 1); // fy
    gKd[2] = K.at<double>(0, 2); // cx
    gKd[3] = K.at<double>(1, 2); // cy

    vector<cv::Point3f> vpts3d;
    vector<cv::Point2f> vpts2d;

    for (cv::DMatch m:vmatches)
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

    cout << "3d-2d pairs: " << vpts3d.size() << endl;

    cv::Mat r, t;
    cv::solvePnP(vpts3d, vpts2d, K, cv::Mat(), r, t, false);
    cv::Mat R;
    cv::Rodrigues(r, R);

    cout << "R " << R << endl;
    cout << "t " << t << endl;

    PoseOptimization(vpts3d, vpts2d, K, R, t);

    CalcuReprojectionCost(vpts3d, vpts2d, K, R, t);

    while(1)
    {
        cv::imshow(" ", showImg);
        cv::waitKey(10);
    }

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
}

/**
 * @brief convert the point in the image plane to the camera plane
 * @param p
 * @param K
 * @return
 */
cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& K)
{
    cv::Point2d pts2d;

    pts2d.x = (p.x - K.at<double>(0, 2))/K.at<double>(0, 0);
    pts2d.y = (p.y - K.at<double>(1, 2))/K.at<double>(1, 1);

    return pts2d;
}

struct ReprojectionError2
{

    ReprojectionError2(double observedx, double observedy)
            : observedx(observedx), observedy(observedy) {}

    template <typename T>
    bool operator()(const T* const Rqcw, const T* const tcw, const T* const Point3D, T* resuduals) const
    {
        T Point[3];
        ceres::QuaternionRotatePoint(Rqcw, Point3D, Point);

        Point[0] += tcw[0];
        Point[1] += tcw[1];
        Point[2] += tcw[2];

        // K[0] fx; K[1] fy; K[2] cx; K[3] cy;
        // in the ceres BA example, it should apply second and fourth order radial distortion
        // but in the usually, the feature point have been distortion in the previous step
        T xp = 529.9*Point[0]/Point[2] + 325.1;
        T yp = 521.0*Point[1]/Point[2] + 249.7;

//        T xp = Point[0]/Point[2];
//        T yp = Point[1]/Point[2];

        resuduals[0] = xp - T(observedx);
        resuduals[1] = yp - T(observedy);

        return true;
    }

    static ceres::CostFunction* Create(const double observedx, const double observedy)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError2, 2, 4, 3, 3>(new ReprojectionError2(observedx, observedy)));
    }

    double observedx;
    double observedy;

}; // struct ReprojectionError2

void PoseOptimization(const vector<cv::Point3f> vPoints3d, const vector<cv::Point2f> vPoints2d,
                      const cv::Mat& K, const cv::Mat& R, const cv::Mat& t)
{
    ceres::Problem problem;
    ceres::LocalParameterization* localParameterization = new ceres::QuaternionParameterization();

    Eigen::Matrix3d Rtemp;
    cv::cv2eigen(R, Rtemp);

    double Rdcw[4];
    double tdcw[4];
    double Kd[4];
    double Point3dw[3];

    Rdcw[0] = Eigen::Quaterniond(Rtemp).w();
    Rdcw[1] = Eigen::Quaterniond(Rtemp).x();
    Rdcw[2] = Eigen::Quaterniond(Rtemp).y();
    Rdcw[3] = Eigen::Quaterniond(Rtemp).z();

    tdcw[0] = t.at<double>(0,0);
    tdcw[1] = t.at<double>(0,1);
    tdcw[2] = t.at<double>(0,2);

//    Kd[0] = K.at<double>(0, 0); // fx
//    Kd[1] = K.at<double>(1, 1); // fy
//    Kd[2] = K.at<double>(0, 2); // cx
//    Kd[3] = K.at<double>(1, 2); // cy

    problem.AddParameterBlock(Rdcw, 4, localParameterization);
    problem.AddParameterBlock(tdcw, 3);

    for (int i = 0; i < vPoints3d.size(); i++)
    {
        ceres::CostFunction* costFunction = ReprojectionError2::Create(vPoints2d[i].x, vPoints2d[i].y);

        Point3dw[0] = vPoints3d[i].x;
        Point3dw[1] = vPoints3d[i].y;
        Point3dw[2] = vPoints3d[i].z;

//        Point3dw[0] = K.at<double>(0, 0)*vPoints3d[i].x/vPoints3d[i].z + K.at<double>(0, 2);
//        Point3dw[1] = K.at<double>(1, 1)*vPoints3d[i].y/vPoints3d[i].z + K.at<double>(1, 2);
//        Point3dw[2] = 1;

        problem.AddResidualBlock(costFunction, nullptr, Rdcw, tdcw, Point3dw);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.3;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << summary.BriefReport() << endl;

    cout << tdcw[0] << endl << tdcw[1] << endl << tdcw[2] << endl;
}

void CalcuReprojectionCost(const vector<cv::Point3f> vPoints3d, const vector<cv::Point2f> vPoints2d,
                           const cv::Mat& K, const cv::Mat& R, const cv::Mat& t)
{
    double cost=0;
    vector<Eigen::Vector2d> vresiduals;
    vresiduals.resize(vPoints3d.size());

    Eigen::Matrix3d Rtemp;
    cv::cv2eigen(R, Rtemp);

    double Rdcw[4];
    double tdcw[4];
    double Point3dw0[3];
    double Point3dw1[3];

    Rdcw[0] = Eigen::Quaterniond(Rtemp).w();
    Rdcw[1] = Eigen::Quaterniond(Rtemp).x();
    Rdcw[2] = Eigen::Quaterniond(Rtemp).y();
    Rdcw[3] = Eigen::Quaterniond(Rtemp).z();

    tdcw[0] = t.at<double>(0,0);
    tdcw[1] = t.at<double>(0,1);
    tdcw[2] = t.at<double>(0,2);

    for (int i = 0; i < vPoints3d.size(); i++)
    {
        Point3dw0[0] = vPoints3d[i].x;
        Point3dw0[1] = vPoints3d[i].y;
        Point3dw0[2] = vPoints3d[i].z;

        ceres::QuaternionRotatePoint(Rdcw, Point3dw0, Point3dw1);

        Point3dw1[0] += tdcw[0];
        Point3dw1[1] += tdcw[1];
        Point3dw1[2] += tdcw[2];

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

    cout << "cost: " << cost << endl;
}