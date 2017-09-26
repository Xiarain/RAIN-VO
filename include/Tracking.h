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

using namespace std;

namespace RAIN_VIO
{

class Tracking
{


public:

    Tracking(const string &strSettingsFile);

    cv::Mat ProcessImage(const cv::Mat &im, const double &timestamp);

    void DetectKeyPoint(const cv::Mat &image, const int numFeatureNeeds);

    void DeleteErrStatus(vector<cv::Point2f> &v, vector<uchar> status);
    void DeleteErrStatus(vector<cv::KeyPoint> &v, vector<uchar> status);
    bool inBorder(const cv::Point2f &pt);
    void RejectWithF(void);

    enum eTrackingState {
        NO_INITIALIZED = 0,
        OK = 1,
        LOST = 2,
        BAD = 3
    };

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

//    cv::Mat prev_img, cur_img, forw_img;
//    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
//    vector<cv::Point2f> n_pts;

    int firstflag;
    list< cv::Point2f > keypoints;

private:

    bool mbFirstImage;
    double mFirstImageTime;
    int numFeatures;
    vector<cv::KeyPoint> mvKeyPoints;
    vector<cv::KeyPoint> mvPreKeyPoints;

    vector<cv::Point2f> mvPointsPts;
    vector<cv::Point2f> mvPrePointsPts;
    vector<cv::Point2f> mvCurPointsPts;
    vector<cv::Point2f> mvNextPointsPts;


}; // class Tracking

} // namespce RAIN_VIO
#endif //RAIN_VIO_TRACKING_H
