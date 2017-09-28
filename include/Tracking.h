//
// Created by rain on 17-9-19.
//

#ifndef RAIN_VIO_TRACKING_H
#define RAIN_VIO_TRACKING_H

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

struct Grid
{
    int x;
    int y;
    float response;
    cv::KeyPoint keypoint;

    // constructed function
    Grid()
    {
        x = 0;
        y = 0;
        response = 0;
    }
};



class Tracking
{


public:

    Tracking(const string &strSettingsFile);
    ~Tracking();

    cv::Mat ProcessImage(const cv::Mat &im, const double &timestamp);

    void DetectKeyPoint(const cv::Mat &image, const int numFeatureNeeds);

    void DeleteErrStatus(vector<cv::Point2f> &v, vector<uchar> status);
    void DeleteErrStatus(vector<cv::KeyPoint> &v, vector<uchar> status);
    void DeleteErrStatus(vector<int> &v, vector<uchar > status);
    bool inBorder(const cv::Point2f &pt);
    void RejectWithF(void);
    void SetMask();

    Camera *mcamera;

    enum eTrackingState {
        NO_INITIALIZED = 0,
        OK = 1,
        LOST = 2,
        BAD = 3
    };

    cv::Mat mMask;
    cv::Mat mImage;
    cv::Mat mNextImage;
    cv::Mat mPreImage;
    cv::Mat mCurImage;

    bool EQUALIZE;
    cv::Mat CmaeraK;
    cv::Mat DistCoef;
    double ImageHeight;
    double ImageWidth;
    double ImageGridHeight;
    double ImageGridWidth;
    vector<Grid> Grids;
    vector<int> PointTrackcnt;

private:

    bool mbFirstImage;
    double mFirstImageTime;
    int numFeatures;
    vector<cv::KeyPoint> mvKeyPoints;
    vector<cv::KeyPoint> mvPreKeyPoints;
    vector<cv::KeyPoint> mvNextKeyPoints;
    vector<cv::KeyPoint> mvCurKeyPoints;

    vector<cv::Point2f> mvPointsPts;
    vector<cv::Point2f> mvPrePointsPts;
    vector<cv::Point2f> mvCurPointsPts;
    vector<cv::Point2f> mvNextPointsPts;


}; // class Tracking

} // namespce RAIN_VIO
#endif //RAIN_VIO_TRACKING_H
