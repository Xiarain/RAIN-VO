//
// Created by rain on 17-12-9.
//

#include "Converter.h"

using namespace RAIN_VIO;

int main(int argv, char* argc[])
{

    // the type of the cv::Mat is very important, if not there is a big problem
    cv::Mat cvmat, r(3, 1, CV_64FC1, cv::Scalar(1));
    Eigen::Matrix3d eigenmat;

    r.at<double>(0, 0) = 0.0031;
    r.at<double>(1, 0) = 0.0033;
    r.at<double>(2, 0) = 0.0034;

    cv::Rodrigues(r, cvmat);

    eigenmat = Converter::toMatrix3d(cvmat);

    std::cout << eigenmat << std::endl;

    return 0;
}

