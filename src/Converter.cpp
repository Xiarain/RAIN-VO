//
// Created by rain on 17-10-19.
//

#include "Converter.h"

namespace RAIN_VIO
{

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j) = (float)m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 3, 3> &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j) = (float)m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 3, 1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
        cvMat.at<float>(i) = (float)m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j) = (float)R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3) = (float)t(i);
    }

    return cvMat.clone();
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = (float)q.x();
    v[1] = (float)q.y();
    v[2] = (float)q.z();
    v[3] = (float)q.w();

    return v;
}

Eigen::Matrix<double, 2, 1> Converter::toVector2d(const cv::Point2f &cvPoint)
{
    Eigen::Matrix<double, 2, 1> v;
    v << cvPoint.x, cvPoint.y;

    return v;
}

cv::Point2f Converter::toCvPoint2f(const Eigen::Vector2d &v)
{
    cv::Point2f cvPoint2f;
    cvPoint2f.x = (float)v[0];
    cvPoint2f.y = (float)v[1];

    return cvPoint2f;
}

cv::Point3f Converter::toCvPoint3f(const Eigen::Vector3d &v)
{
    cv::Point3f cvPoint3f;
    cvPoint3f.x = (float)v[0];
    cvPoint3f.y = (float)v[1];
    cvPoint3f.z = (float)v[2];

    return cvPoint3f;
}

/**
 * @brief euler angle； yaw, pitch, roll
 * @param q
 * @return
 */
Eigen::Vector3d Converter::toEuler(const Eigen::Quaterniond &q)
{
    double yaw, pitch, roll;
    double r1 = 2*(q.w()*q.x() + q.y()*q.z());
    double r2 = 1 - 2*(q.x()*q.x() + q.y()*q.y());
    double r3 = 2*(q.w()*q.y() - q.z()*q.x());
    double r4 = 2*(q.w()*q.z() + q.x()*q.y());
    double r5 = 1 - 2*(q.y()*q.y() + q.z()*q.z());

    roll = atan2(r1, r2);
    pitch = asin(r3);
    yaw = atan2(r4,r5);

    Eigen::Vector3d euler(yaw,pitch,roll);

    return euler;
}

} // namesapce RAIN_VIO