//
// Created by rain on 17-12-9.
//

#include "Converter.h"

using namespace std;

int main(int argv, char* argc[])
{

    // the type of the cv::Mat is very important, if not there is a big problem
    cv::Mat cvmat, r(3, 1, CV_64FC1, cv::Scalar(1));
    Eigen::Matrix3d eigenmat;

    r.at<double>(0, 0) = 0.0031;
    r.at<double>(1, 0) = 0.0033;
    r.at<double>(2, 0) = 0.0034;

    cv::Rodrigues(r, cvmat);

    eigenmat = RAIN_VIO::Converter::toMatrix3d(cvmat);

    cout << "eigen rotation matrix: " << endl;
    cout << eigenmat << endl;

    cout << "converter eigen rotation matrix to euler: "<< endl;
    cout << RAIN_VIO::Converter::toEuler(eigenmat).transpose() << endl;

    cout << "converter quaternion to euler: "<< endl;
    cout << RAIN_VIO::Converter::toEuler(Eigen::Quaterniond(eigenmat)).transpose() << endl;

    Eigen::Vector3d t(1.0, 0., 2.0);

    Eigen::Quaterniond Rq(eigenmat);

    cout << "quaternion: " << endl;
    cout << Rq.coeffs().transpose() << endl;

    cout << "quaternion rotate the vector: "<< endl;
    Eigen::Vector3d t2 = Rq * t;
    cout << t2.transpose() << endl;

    cout << "quaternion to rotation matrix and rotate the vector: "<< endl;
    cout << (Rq.toRotationMatrix()*t).transpose() << endl;

    return 0;
}

