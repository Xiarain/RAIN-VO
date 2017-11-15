//
// Created by rain on 17-10-5.
//

#ifndef RAIN_VIO_FEATURE_H
#define RAIN_VIO_FEATURE_H

#include <string>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include "Camera.h"

using namespace std;

namespace RAIN_VIO
{

class Camera;

class Feature
{


public:

    Feature(Camera *pCamera, const string &strSettingsFile, const int nWindowSize);
    ~Feature();

    bool ProcessImage(const cv::Mat &im, const double &timestamp);
    void DetectKeyPoint(const cv::Mat &image, const int numFeatureNeeds);
    bool inBorder(const cv::Point2f &pt);
    void RejectWithF(void);
    void SetMask();
    void UpdateKeyPointID();
    vector<cv::Point2f> UndistoredPoints();
    cv::Mat UndistoredImage(const cv::Mat image);

    Camera *mpCamera;

    enum eTrackingState {
        NO_INITIALIZED = 0,
        OK = 1,
        LOST = 2,
        BAD = 3
    };

    int mnWindowSize;
    
    cv::Mat mMask;
    cv::Mat mImage;
    cv::Mat mNextImage;
    cv::Mat mPreImage;
    cv::Mat mCurImage;
    cv::Mat mNextImageShow, mCurImageShow;

    bool EQUALIZE;
    cv::Mat CmaeraK;
    cv::Mat DistCoef;
    double ImageHeight;
    double ImageWidth;
    double ImageGridHeight;
    double ImageGridWidth;
    vector<int> mvPointTrackcnt;
    vector<uint> mvPointTrackID;

    vector<cv::Point2f> mvPointsPts;
    vector<cv::Point2f> mvPrePointsPts;
    vector<cv::Point2f> mvCurPointsPts;
    vector<cv::Point2f> mvNextPointsPts;

private:

    bool mbFirstImage;
    double mFirstImageTime;
    int mnumFeatures;
    int minDist;
    uint mIDcnt;
    int mFeatureShow;
    vector<cv::KeyPoint> mvKeyPoints;
    vector<cv::KeyPoint> mvPreKeyPoints;
    vector<cv::KeyPoint> mvNextKeyPoints;
    vector<cv::KeyPoint> mvCurKeyPoints;
}; // class Feature

} // namespce RAIN_VIO


#endif //RAIN_VIO_FEATURE_H
