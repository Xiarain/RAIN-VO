//
// Created by rain on 17-9-19.
//

#include "Tracking.h"

#include "TicToc.h"
#include <algorithm>

#define ORBKEYPOINT

namespace RAIN_VIO
{

template <typename T>
static void DeleteErrStatus(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < v.size(); i++)
    {
        if (status[i])
            v[j++] = v[i];
    }

    v.resize(j);
}

Tracking::Tracking(const string &strSettingsFile)
{
    EQUALIZE = true;
    mbFirstImage = true;
    mFirstImageTime = 0;

    mcamera = new Camera(strSettingsFile);

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
    DistCoef.at<float>(0) = fsSettings["Camera.k1"];
    DistCoef.at<float>(1) = fsSettings["Camera.k2"];
    DistCoef.at<float>(2) = fsSettings["Camera.p1"];
    DistCoef.at<float>(3) = fsSettings["Camera.p2"];

    ImageHeight = fsSettings["Camera.height"];
    ImageWidth = fsSettings["Camera.width"];

    ImageGridHeight = fsSettings["ImageGridHeight"];
    ImageGridWidth = fsSettings["ImageGridWidth"];

    numFeatures = fsSettings["ORBextractor.numFeatures"];
    minDist = fsSettings["ORBextractor.minDist"];

}

Tracking::~Tracking()
{
    delete(mcamera);
}

cv::Mat Tracking::ProcessImage(const cv::Mat &image, const double &timestamp)
{
    // the image for the display
    mNextImageShow = image.clone();
    cv::cvtColor(mNextImageShow, mNextImageShow, CV_GRAY2RGB);

    // Turn the color image to the gray image
    if (image.channels() == 3) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
    }

    // histogram equalization for the image
    if (EQUALIZE == true) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->apply(image.rowRange(0, ImageHeight), image);
    }
    // the process for the timestamp of the image
    if (mbFirstImage == true) {
        mFirstImageTime = timestamp;
        mbFirstImage = false;
    }

    if (mNextImage.empty()) {
        mPreImage = mCurImage = mNextImage = image;
    } else {
        mNextImage = image;
    }

    mvNextPointsPts.clear();

    if (mvCurPointsPts.size() > 0) {

        vector<uchar> status;
        vector<float> err;

        // mvNextPoints
        cv::calcOpticalFlowPyrLK(mCurImage, mNextImage, mvCurPointsPts, mvNextPointsPts, status, err, cv::Size(21, 21),
                                 3);

        for (int i = 0; i < int(mvNextPointsPts.size()); i++)
            if (status[i] && !inBorder(mvNextPointsPts[i]))
                status[i] = 0;

        DeleteErrStatus(mvPrePointsPts, status);
        DeleteErrStatus(mvCurPointsPts, status);
        DeleteErrStatus(mvNextPointsPts, status);
        DeleteErrStatus(PointTrackcnt, status);
    }

    RejectWithF();
//  the number of times to track plus one
    for (auto &n : PointTrackcnt)
    {
        n++;
    }


    SetMask();
    // the feature number should be stained in a certain number
    // so the number is too few, it should detect more feature point, if not, the new feature point should been discarded.
    int n_max_cnt = numFeatures - static_cast<int>(mvNextPointsPts.size());
    if (n_max_cnt > 0)
    {

        TicTOC timefeature;
        // detect the feature point (mvKeyPoints, mvPointsPts)
        DetectKeyPoint(mNextImage, numFeatures - mvNextPointsPts.size()); // n_pts

#if 0
        mvPointsPts.clear();
        cv::goodFeaturesToTrack(mNextImage, mvPointsPts, n_max_cnt, 0.1, 30, mMask);
#endif

        cout << " the time of the detect the feature point:" << timefeature.toc() << endl;
    }
    else
    {
        mvPointsPts.clear();
        mvKeyPoints.clear();
    }

    for (auto &p : mvPointsPts)
    {
        mvNextPointsPts.push_back(p);

        // the number of the tracked of the new point is one
        PointTrackcnt.push_back(1);
    }


    mPreImage = mNextImage;
    mvPrePointsPts = mvNextPointsPts;

    if (!mCurImageShow.empty())
    {
        for (int i = 0; i < mvCurPointsPts.size(); i++)
        {
            double len = min(1.0, 1.0*PointTrackcnt[i]/20);
            cv::circle(mCurImageShow, mvCurPointsPts[i], 2, cv::Scalar(255*(1 - len), 0, 255*len), 2);
        }

        cv::imshow("", mCurImageShow);
        cv::waitKey(20);
    }

    mCurImageShow = mNextImageShow;
    mCurImage = mNextImage;
    mvCurPointsPts = mvNextPointsPts;

    cout << "the number of the feature point " << mvNextPointsPts.size() << endl;

    return cv::Mat::eye(4, 4, CV_32F);
}

void Tracking::DetectKeyPoint(const cv::Mat &image, const int numFeatureNeeds)
{
    vector<cv::KeyPoint> vKeyPoints;

    mvKeyPoints.clear();
    mvPointsPts.clear();

#ifdef ORBKEYPOINT
    cv::Ptr<cv::FeatureDetector> ORBDetctor = cv::ORB::create(numFeatureNeeds, 1.2, 1, 10, 0, 2, cv::ORB::FAST_SCORE, 10, 40);
    ORBDetctor->detect(image, mvKeyPoints, mMask);
#else
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(30, true, cv::FastFeatureDetector::TYPE_9_16);
    detector->detect( image, mvKeyPoints, mMask);
#endif

    const int CellSize = minDist+30;
    int GridnCols = ceil(static_cast<int>(ImageWidth / CellSize));
    int GridnRows = ceil(static_cast<int>(ImageHeight / CellSize));

    vector<float> GridOccupancy;
    GridOccupancy.resize(GridnCols*GridnRows);

    fill(GridOccupancy.begin(), GridOccupancy.end(), 0);

    sort(mvKeyPoints.begin(), mvKeyPoints.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b){
        return a.response > b.response;
    });

    int i = 0;
    if (mvKeyPoints.size() > 0)
    {
        for (auto &Points : mvKeyPoints)
        {
            int ceilx = static_cast<int>(Points.pt.x/CellSize);
            int ceily = static_cast<int>(Points.pt.y/CellSize);

            if (GridOccupancy[ceilx + ceily*GridnRows] == 0)
            {
                GridOccupancy[ceilx + ceily*GridnRows] = Points.response;
                mvKeyPoints[i++] = Points;
            }
        }
    }
    mvKeyPoints.resize(i);

    for (auto &KeyPoints : mvKeyPoints)
    {
        mvPointsPts.push_back(KeyPoints.pt);
    }

}

bool Tracking::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < ImageWidth - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ImageHeight - BORDER_SIZE;
}


void Tracking::RejectWithF(void)
{
    vector<uchar> status;
    vector<cv::Point2f> vunNextPointsPts, vunPrePointsPts;

    vunNextPointsPts = mvNextPointsPts;
    vunPrePointsPts = mvPrePointsPts;
    vunNextPointsPts.resize(vunPrePointsPts.size());
    vunPrePointsPts.resize(vunPrePointsPts.size());

// OpenCV Fundamental matrix
#if 1
    // Fill matrix with points
    if (vunPrePointsPts.size() > 8)
    {
        cv::Mat mat(vunPrePointsPts.size(),2,CV_32F);
        for(int i = 0; i < vunPrePointsPts.size(); i++)
        {
            mat.at<float>(i,0) = vunPrePointsPts[i].x;
            mat.at<float>(i,1) = vunPrePointsPts[i].y;

//            cout << vunPrePointsPts[i].x << " " << vunPrePointsPts[i].y << endl;
        }
        mat=mat.reshape(2);
        cv::undistortPoints(mat, mat, CmaeraK, DistCoef, cv::Mat(), CmaeraK);
        mat=mat.reshape(1);
        cout << "after undistortion" << endl;

        for (int i = 0; i <vunPrePointsPts.size(); i++)
        {
            vunPrePointsPts[i].x = mat.at<float>(i,0);
            vunPrePointsPts[i].y = mat.at<float>(i,1);

//            cout << vunPrePointsPts[i].x << " " << vunPrePointsPts[i].y << endl;
        }

        for(int i = 0; i < vunNextPointsPts.size(); i++)
        {
            mat.at<float>(i,0) = vunNextPointsPts[i].x;
            mat.at<float>(i,1) = vunNextPointsPts[i].y;
        }
        mat=mat.reshape(2);
        cv::undistortPoints(mat, mat, CmaeraK, DistCoef, cv::Mat(), CmaeraK);
        mat=mat.reshape(1);

        for (int i = 0; i <vunNextPointsPts.size(); i++)
        {
            vunNextPointsPts[i].x = (int)mat.at<float>(i,0);
            vunNextPointsPts[i].y = (int)mat.at<float>(i,1);
        }
    }

#else

    const double FOCAL_LENGTH = 460.0;

    for (int i = 0; i < mvPrePointsPts.size(); i++)
    {
        Eigen::Vector3d tmpP;

        mcamera->LiftProjective(Eigen::Vector2d(mvPrePointsPts[i].x, mvPrePointsPts[i].y), tmpP);
        tmpP[0] = FOCAL_LENGTH*tmpP[0]/tmpP[2] + ImageWidth/2.0;
        tmpP[1] = FOCAL_LENGTH*tmpP[1]/tmpP[2] + ImageHeight/2.0;
        vunPrePointsPts[i] = cv::Point2f(tmpP[0], tmpP[1]);

        mcamera->LiftProjective(Eigen::Vector2d(mvNextPointsPts[i].x, mvNextPointsPts[i].y), tmpP);
        tmpP[0]  = FOCAL_LENGTH*tmpP[0]/tmpP[2] + ImageWidth/2.0;
        tmpP[1]  = FOCAL_LENGTH*tmpP[1]/tmpP[2] + ImageHeight/2.0;
        vunNextPointsPts[i] = cv::Point2f(tmpP[0], tmpP[1]);
    }

#endif
    // the solution of the fundamental matrix needs eight points
    if (mvNextPointsPts.size() > 8 && mvPrePointsPts.size() > 8)
    {
//        vector<cv::Point2f> vunNextPointsPts(mvNextPointsPts.size()), vunPrePointsPts(mvPreKeyPoints.size());
        // the maximum distance from a point to an epipolar line in pixels,
        cv::findFundamentalMat(vunPrePointsPts, vunNextPointsPts, cv::FM_RANSAC, 2.5, 0.99, status);

        DeleteErrStatus(mvPrePointsPts, status);
        DeleteErrStatus(mvNextPointsPts, status);
        DeleteErrStatus(mvCurPointsPts, status);
        DeleteErrStatus(PointTrackcnt, status);
    }
    else
    {
        cout << "the number of the points is too little when solve the fundamental matrix" << endl;
    }

    int i = 0;
    for (auto &v:status)
    {
        if (v == 0)
            i++;
    }
    cout << "F:" << status.size() << " " << i << endl;


}

void Tracking::SetMask()
{
    mMask = cv::Mat(ImageHeight, ImageWidth, CV_8UC1, cv::Scalar(255));

    vector<pair<int ,cv::Point2f>> PointsCnt;

    for (int i = 0; i < mvNextPointsPts.size(); i++)
    {
        PointsCnt.push_back(make_pair(PointTrackcnt[i], mvNextPointsPts[i]));
    }

    sort(PointsCnt.begin(), PointsCnt.end(), [](const pair<int, cv::Point2f> &a, const pair<int, cv::Point2f> &b)
         {
             return a.first > b.first;
         });

    mvNextPointsPts.clear();
    PointTrackcnt.clear();

    for (auto &Point : PointsCnt)
    {
        if (mMask.at<uchar>(Point.second) == 255) // !!! must have the <uchar>
        {
            mvNextPointsPts.push_back(Point.second);
            PointTrackcnt.push_back(Point.first);
            cv::circle(mMask, Point.second, minDist, 0, -1);
        }
    }
}

} // namespace RAIN_VIO
