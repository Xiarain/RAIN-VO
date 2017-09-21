//
// Created by rain on 17-9-19.
//

#include "../include /Tracking.h"

#define FAST


namespace RAIN_VIO
{

void Tracking::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);

    }
}
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];

    // 根据状态好的变量重新配置大小
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

Tracking::Tracking(const string &strSettingsFile)
{
    EQUALIZE = false;
    mbFirstImage = true;
    mFirstImageTime = 0;


    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at " << strSettingsFile << endl;
        exit(-1);
    }

    CmaeraK = cv::Mat::eye(3, 3, CV_32F);
    CmaeraK.at<float>(0, 0) = fsSettings["Camera.fx"];
    CmaeraK.at<float>(1, 1) = fsSettings["Camera.fy"];
    CmaeraK.at<float>(0, 2) = fsSettings["Camera.cx"];
    CmaeraK.at<float>(1, 2) = fsSettings["Camera.cy"];

    DistCoef = cv::Mat::zeros(4, 1, CV_32F);
    float temp = fsSettings["Camera.k1"];
    DistCoef.at<float>(0, 0) = fsSettings["Camera.k1"];
    DistCoef.at<float>(1, 0) = fsSettings["Camera.k2"];
    DistCoef.at<float>(2, 0) = fsSettings["Camera.p1"];
    DistCoef.at<float>(3, 0) = fsSettings["Camera.p2"];

    ImageHeight = fsSettings["Camera.height"];
    ImageWidth = fsSettings["Camera.width"];

    ImageGridHeight = fsSettings["ImageGridHeight"];
    ImageGridWidth = fsSettings["ImageGridWidth"];

    firstflag = 1;
}

cv::Mat Tracking::ProcessImage(const cv::Mat &image, const double &timestamp)
{



    cv::Mat img, ImageShow;

    img = image;

    ImageShow = image.clone();
    cv::cvtColor(ImageShow, ImageShow, CV_GRAY2RGB);

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    // 上一幅图像中的2D特征点已经找到了
    if (cur_pts.size() > 0)
    {

        vector<uchar> status;
        vector<float> err;


        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        // 前一帧追踪2D点
        reduceVector(prev_pts, status);

        // 当前帧追踪2D点
        reduceVector(cur_pts, status);

        // 下一帧追踪2D点
        reduceVector(forw_pts, status);

    }

    // 只要相机的图像帧时间戳没有问题，这个帧就会被发送
    if (1)
    {

        int n_max_cnt = 300 - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {

//            cv::goodFeaturesToTrack(forw_img, n_pts, 300 - forw_pts.size(), 0.1, 30, cv::Mat(ImageHeight, ImageWidth, CV_8UC1, cv::Scalar(255)));
            DetectKeyPoint(image);

            n_pts.resize(mvKeyPoints.size());
            for (int i = 0; i < mvKeyPoints.size(); i++)
            {
                n_pts[i] = mvKeyPoints[i].pt;
            }

        }
       else
            n_pts.clear();

        addPoints();
        prev_img = forw_img;
        prev_pts = forw_pts;
    }
    cur_img = forw_img;
    cur_pts = forw_pts;


    for (int i = 0; i < forw_pts.size(); i++)
    {
        cv::circle(ImageShow, forw_pts[i], 2, cv::Scalar(0, 0, 255), 2);
    }

    cv::imshow("", ImageShow);
    cv::waitKey(20);


//    cv::Mat ImageShow;
//
//    if (mbFirstImage == true)
//    {
//        mFirstImageTime = timestamp;
//        mbFirstImage = false;
//    }
//
//
//    image.copyTo(mImage);
//
//    // Turn the color image to the gray image
//    if (mImage.channels() == 3)
//    {
//        cv::cvtColor(mImage, mImage, CV_RGB2BGR);
//    }
//
//    // histogram equalization for the image
//    if (EQUALIZE == true)
//    {
//        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
//        clahe->apply(mImage.rowRange(0, ImageHeight), mImage);
//    }
//
//    // initialize the data
////    if (mPreImage.empty())
////    {
////        mPreImage = mImage;
////    }
//
//#ifdef FAST
//
//    DetectKeyPoint(mImage);
//
//    mImage.copyTo(ImageShow);
//    cv::cvtColor(ImageShow, ImageShow, CV_GRAY2RGB);
//
//
//#else
//
//    vector<cv::Point2f> n_pts;
//    cv::goodFeaturesToTrack(mImage, n_pts, 300, 0.1, 30,cv::Mat(ImageHeight, ImageWidth, CV_8UC1, cv::Scalar(255)));
//
//    mImage.copyTo(ImageShow);
//    cv::cvtColor(ImageShow, ImageShow, CV_GRAY2RGB);
//    for (int i = 0; i < n_pts.size(); i++)
//    {
//
//        cv::circle(ImageShow, n_pts[i], 2, cv::Scalar(0, 0, 255), 2);
//    }
//
//#endif
//
////    cout << "光流之前" << mvKeyPoints.size() << " ";
//
//
//
//    mvKeyPointspts.clear();
//    if (!mvPreKeyPointspts.empty())
//    {
//        vector<uchar> status;
//        vector<float> err;
//
//        cv::calcOpticalFlowPyrLK(mPreImage, mImage, mvPreKeyPointspts, mvKeyPointspts, status, err, cv::Size(21, 21), 3);
////        cout << mvPreKeyPointspts[0].x << mvPreKeyPointspts[0].y << endl;
//        int sum = 0;
//
//        for (int i = 0; i < mvKeyPointspts.size(); i++)
//        {
////            if (status[i] && !inBorder(mvKeyPointspts[i]))
////                status[i] = 0;
////            else
////            {
////                status[i] = 1;
////                sum = sum + 1;
////            }
//
////            for (int i = 0; i < err.size(); i++)
////            {
////                cout << err[i] << endl;
////            }
//
//        }
////        cout << sum;
////        DeleteErrStatus(mvKeyPointspts, status);
////        DeleteErrStatus(mvKeyPoints, status);
//    }
//
//    cout << mvPreKeyPointspts.size() << mvKeyPointspts.size() << endl;
//
////    cout << "光流之后" << mvKeyPoints.size() << endl;
//
//#ifdef FAST
//    for (int i = 0; i < mvKeyPoints.size(); i++)
//    {
//
//        cv::circle(ImageShow, mvKeyPoints[i].pt, 2, cv::Scalar(0, 0, 255), 2);
//    }
//
//    mvKeyPointspts.resize(mvKeyPoints.size());
//
//    for (int i = 0; i < mvKeyPoints.size(); i++)
//    {
//        mvKeyPointspts[i] = mvKeyPoints[i].pt;
//    }
//    //
//    mvKeyPoints.clear();
//#endif
//
//
//
//
//    mPreImage = mImage.clone();
//
////    mvPreKeyPointspts.resize(mvKeyPointspts.size());
//
//    mvPreKeyPointspts.assign(mvKeyPointspts.begin(), mvKeyPointspts.end());
////    mvPreKeyPointspts = mvKeyPointspts;
//
////    cv::imshow("", ImageShow);
//    cv::waitKey(20);
//

    return cv::Mat::eye(4, 4, CV_32F);
}

void Tracking::DetectKeyPoint(const cv::Mat &image)
{
    vector<cv::KeyPoint> KeyPoints;
    int x0;
    int y0;
#if 0
    // the maximun number of feature to retatin: 500
    // pyramid decimation ration: 1.2
    // the numbers of pyramid levels 8
    // This is size of the border where the features are not detected: 30
    // It should be 0 in the current implementation: 0
    // The number of points that produce each element of the oriented BRIEF descriptor: 2
    // Harris algorithm is used to rank features:
    // size of the patch used by the oriented BRIEF descriptor: 31
    // fastThreshold 20
    cv::Ptr<cv::FeatureDetector> ORBDetctor = cv::ORB::create(1, 1.2, 1, 30, 0, 2, cv::ORB::FAST_SCORE, 30, 100);

    // detect the ORB keypoint
    // Camera.width: 752
    // Camera.height: 480s
    // ImageHeight: 24
    for (int i = 0; i < (ImageHeight/ImageGridHeight); i++)
        for (int j = 0; j < (ImageWidth/ImageGridWidth); j++)
        {
            if ( (i+1)*ImageGridHeight > ImageHeight || (j+1)*ImageGridWidth > ImageWidth)
                break;

            x0 = j*ImageGridHeight;
            y0 = i*ImageGridWidth;
            ORBDetctor->detect(image(cv::Rect(x0, y0, ImageGridHeight, ImageGridWidth)), KeyPoints);

            for (int k = 0; k < KeyPoints.size(); k++)
            {

                KeyPoints[k].pt.x = KeyPoints[k].pt.x + x0;
                KeyPoints[k].pt.y = KeyPoints[k].pt.y + y0;
            }

            mvKeyPoints.insert(mvKeyPoints.end(), KeyPoints.begin(), KeyPoints.end());
        }

#endif

#if 0
    cv::Ptr<cv::FeatureDetector> ORBDetctor = cv::ORB::create(300, 1.2, 1, 10, 0, 2, cv::ORB::FAST_SCORE, 10, 30);
    ORBDetctor->detect(image, mvKeyPoints);
#endif

#if 1
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(30, true, cv::FastFeatureDetector::TYPE_9_16);
    detector->detect( image, mvKeyPoints );
#endif
}

void Tracking::DeleteErrStatus(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < v.size(); i++)
    {
        if (status[i])
            v[j++] = v[i];
    }

}
void Tracking::DeleteErrStatus(vector<cv::KeyPoint> &v, vector<uchar > status)
{
    int j = 0;
    for (int i = 0; i < v.size(); i++)
    {
        if (status[i] == 0)
            v[j++] = v[i];
    }

}

bool Tracking::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < ImageWidth - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ImageHeight - BORDER_SIZE;
}

} // namespace RAIN_VIO
