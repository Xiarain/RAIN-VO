//
// Created by rain on 17-9-17.
//

#ifndef RAIN_VIO_FRAME_H
#define RAIN_VIO_FRAME_H

#include "Tracking.h"
#include "Feature.h"
#include <iostream>
#include <vector>

using namespace std;

namespace RAIN_VIO
{

class Feature;

class Frame
{

public:
    int mnWindowSize;

    cv::Mat mImageShow;

    vector<pair<uint, Eigen::Vector3d> > mvFraFeatures; // ID and point vector<pair<int, Eigen::Vector3d>>
    vector<cv::Point2f> mvFraPointsPts;
    vector<uint> mvFraPointsID;
    vector<int> mvFraPointsCnt;

    Frame(const string &strSettingsFile, const int nWindowSize);
    ~Frame();
    void DetectKeyPoint(const cv::Mat &image, const double &TimeStamps);

private:
    Feature *mpfeature;
    double ImageHeight;
    double ImageWidth;

};

}



#endif //RAIN_VIO_FRAME_H
