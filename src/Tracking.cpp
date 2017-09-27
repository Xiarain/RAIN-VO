//
// Created by rain on 17-9-19.
//

#include "Tracking.h"

#include "TicToc.h"
#include <algorithm>

#define FAST


namespace RAIN_VIO
{

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

    firstflag = 1;
}

Tracking::~Tracking()
{
    delete(mcamera);
}

cv::Mat Tracking::ProcessImage(const cv::Mat &image, const double &timestamp)
{

    cv::Mat ImageShow;

    // Turn the color image to the gray image
    if (image.channels() == 3)
    {
        cv::cvtColor(image, image, CV_RGB2BGR);
    }

    // the image for the display
    ImageShow = image.clone();
    cv::cvtColor(ImageShow, ImageShow, CV_GRAY2RGB);

    // histogram equalization for the image
    if (EQUALIZE == true)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->apply(image.rowRange(0, ImageHeight), image);
    }
    // the process for the timestamp of the image
    if (mbFirstImage == true)
    {
        mFirstImageTime = timestamp;
        mbFirstImage = false;
    }


    if (mNextImage.empty())
    {
        mPreImage = mCurImage = mNextImage = image;
    }
    else
    {
        mNextImage = image;
    }


    // the feature number should be stained in a certain number
    // so the number is too few, it should detect more feature point, if not, the new feature point should been discarded.
    int n_max_cnt = numFeatures - static_cast<int>(mvCurPointsPts.size());
    if (n_max_cnt > 0)
    {

        TicTOC timefeature;
        // detect the feature point (mvKeyPoints, mvPointsPts)
        DetectKeyPoint(image, n_max_cnt); // n_pts

#if 0
        cv::goodFeaturesToTrack(mNextImage, mvPointsPts, n_max_cnt, 0.1, 30, cv::Mat(ImageHeight, ImageWidth, CV_8UC1, cv::Scalar(255)));
#endif

        cout << " the time of the detect the feature point:" << timefeature.toc() << endl;

    }
    else
    {
        mvPointsPts.clear();
        mvKeyPoints.clear();
    }

    mvNextPointsPts.clear();
    mvNextKeyPoints.clear();

    // 上一幅图像中的2D特征点已经找到了
    if (mvCurPointsPts.size() > 0)
    {

        vector<uchar> status;
        vector<float> err;

        // mvNextPoints
        cv::calcOpticalFlowPyrLK(mCurImage, mNextImage, mvCurPointsPts, mvNextPointsPts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(mvNextPointsPts.size()); i++)
            if (status[i] && !inBorder(mvNextPointsPts[i]))
                status[i] = 0;

        DeleteErrStatus(mvPrePointsPts, status);
        DeleteErrStatus(mvPreKeyPoints, status);

        DeleteErrStatus(mvCurPointsPts, status);
        DeleteErrStatus(mvCurKeyPoints, status);

        DeleteErrStatus(mvNextPointsPts, status);
        DeleteErrStatus(mvNextKeyPoints, status);
    }

    // reject some points by the fundamental matrix
    RejectWithF();

    mPreImage = mNextImage;
    mvPrePointsPts = mvNextPointsPts;
    mvPreKeyPoints = mvNextKeyPoints;

    mCurImage = mNextImage;
    mvCurPointsPts = mvNextPointsPts;
    mvCurKeyPoints = mvNextKeyPoints;

    // add the new feature point to the next feature vector
    for (auto p : mvPointsPts)
    {
        mvCurPointsPts.push_back(p);
    }

    for (auto KeyPoint : mvKeyPoints)
    {
        mvCurKeyPoints.push_back(KeyPoint);
    }

    mvPointsPts.clear();
    mvKeyPoints.clear();

    for (int i = 0; i < mvNextPointsPts.size(); i++)
    {
        cv::circle(ImageShow, mvNextPointsPts[i], 2, cv::Scalar(0, 0, 255), 2);
    }

    cout << "the number of the feature point " << mvNextPointsPts.size() << endl;

    cv::imshow("", ImageShow);
    cv::waitKey(20);

    return cv::Mat::eye(4, 4, CV_32F);
}

void Tracking::DetectKeyPoint(const cv::Mat &image, const int numFeatureNeeds)
{
    vector<cv::KeyPoint> vKeyPoints;

#if 1
    int x0;
    int y0;
    // the maximun number of feature to retatin: 500
    // pyramid decimation ration: 1.2
    // the numbers of pyramid levels 8
    // This is size of the border where the features are not detected: 30
    // It should be 0 in the current implementation: 0
    // The number of points that produce each element of the oriented BRIEF descriptor: 2
    // Harris algorithm is used to rank features:
    // size of the patch used by the oriented BRIEF descriptor: 31
    // fastThreshold 20
    cv::Ptr<cv::FeatureDetector> ORBDetctor = cv::ORB::create(1, 1.2, 3, 15, 0, 2, cv::ORB::FAST_SCORE, 20, 30);

    const int CellSize = 70;
    const int GridnCols = ceil(static_cast<double>(ImageWidth/CellSize)); // 26
    const int GridnRows = ceil(static_cast<double>(ImageHeight/CellSize)); // 16


    vector<vector<cv::KeyPoint> > GridKeyPoint(GridnCols);

    Grids.resize(GridnRows*GridnCols);

    for (int i = 0; i < GridnRows; i++)
        for (int j = 0; j < GridnCols; j++)
        {
//            GridOccupancy[i][j] = 0;
            Grids[j + i*GridnRows].x = i;
            Grids[j + i*GridnRows].y = j;
            Grids[j + i*GridnRows].response = 0;
        }


    int k = 0;
    for (auto Points : mvNextKeyPoints)
    {
        int ceilx = ceil(static_cast<double>(Points.pt.x/CellSize));
        int ceily = ceil(static_cast<double>(Points.pt.y/CellSize));

//        if (Points.response > GridOccupancy[ceily][ceilx])
//        {
//            GridOccupancy[ceily][ceilx] = Points.response;
//            GridKeyPoint[ceily][ceilx] = Points;
//        }
        if (Points.response > Grids[ceilx + ceily*GridnRows].response)
        {
            // the coordinate of the keypoint
            Grids[ceilx + ceily*GridnRows].x = Points.pt.x;
            Grids[ceilx + ceily*GridnRows].y = Points.pt.y;
            Grids[ceilx + ceily*GridnRows].response = Points.response;

            k++;
            mvNextKeyPoints[k] = Points;
        }
    }
    mvNextKeyPoints.resize(k);

    // detect the ORB keypoint
    for (int i = 0; i < GridnRows; i++) // y
        for (int j = 0; j < GridnCols; j++) // x
        {

            if ( (j+1)*CellSize > ImageWidth || (i+1)*CellSize > ImageHeight)
                break;

            x0 = j*CellSize;
            y0 = i*CellSize;

            if (!(Grids[j + i*GridnRows].response > 0))
            {
                ORBDetctor->detect(image(cv::Rect(x0, y0, CellSize, CellSize)), vKeyPoints);

                for (int k = 0; k < vKeyPoints.size(); k++)
                {
                    vKeyPoints[k].pt.x = vKeyPoints[k].pt.x + x0;
                    vKeyPoints[k].pt.y = vKeyPoints[k].pt.y + y0;
                }

                mvKeyPoints.insert(mvKeyPoints.end(), vKeyPoints.begin(), vKeyPoints.end());
            }
            else
                vKeyPoints.clear();
        }
    // sort the keypoints, find the high response of the keypoint
    sort(vKeyPoints.begin(), vKeyPoints.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b)
    {
        return a.response > b.response;
    });

    int count = 0;
    for (auto KeyPoints : mvKeyPoints)
    {
        mvPointsPts.push_back(KeyPoints.pt);

        count++;
        if (!(count < numFeatureNeeds))
        {
            mvKeyPoints.resize(count);
            return;
        }
    }

#endif

#if 0
    cv::Ptr<cv::FeatureDetector> ORBDetctor = cv::ORB::create(numFeatureNeeds, 1.2, 1, 30, 0, 2, cv::ORB::FAST_SCORE, 10, 30);
    ORBDetctor->detect(image, mvKeyPoints);

    for (auto KeyPoints : mvKeyPoints)
    {
        mvPointsPts.push_back(KeyPoints.pt);
    }
#endif

#if 0
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(30, true, cv::FastFeatureDetector::TYPE_9_16);
    detector->detect( image, mvKeyPoints );

    sort(mvKeyPoints.begin(), mvKeyPoints.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b)
    {
        return a.response > b.response;
    });

    for (int i = 0; i < numFeatureNeeds; i++)
    {
        mvPointsPts.push_back(mvKeyPoints[i].pt);
    }
#endif

//    const int CellSize = 30;
//    int GridnCols = ceil(static_cast<double>(ImageWidth / CellSize));
//    int GridnRows = ceil(static_cast<double>(ImageHeight / CellSize));
//
//    vector<float > GridOccupancy;
//    GridOccupancy.resize(GridnCols*GridnRows);
//
//    fill(GridOccupancy.begin(), GridOccupancy.end(), 0);
//
//    int i = 0;
//
//    for (auto KeyPoints : mvKeyPoints)
//    {
//        const int k = static_cast<int> (KeyPoints.pt.y/CellSize)*GridnCols + static_cast<int>(KeyPoints.pt.x/CellSize);
//
//        if (KeyPoints.response > GridOccupancy[k])
//        {
//            GridOccupancy[k] = KeyPoints.response;
//
//            mvKeyPoints[i++] = KeyPoints;
//        }
//
//    }
//
//    mvKeyPoints.resize(i);


}

void Tracking::DeleteErrStatus(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < v.size(); i++)
    {
        if (status[i])
            v[j++] = v[i];
    }
    v.resize(j);

}
void Tracking::DeleteErrStatus(vector<cv::KeyPoint> &v, vector<uchar > status)
{
    int j = 0;
    for (int i = 0; i < v.size(); i++)
    {
        if (status[i] == 0)
            v[j++] = v[i];
    }
    v.resize(j);
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
#if 0
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

    for (int i = 0; i < vunPrePointsPts.size(); i++)
    {
        Eigen::Vector3d tmpP;

        mcamera->LiftProjective(Eigen::Vector2d(vunPrePointsPts[i].x, vunPrePointsPts[i].y), tmpP);
        tmpP[0] = FOCAL_LENGTH*tmpP[0]/tmpP[2] + ImageWidth/2.0;
        tmpP[1] = FOCAL_LENGTH*tmpP[1]/tmpP[2] + ImageWidth/2.0;
        vunPrePointsPts[i] = cv::Point2f(tmpP[0], tmpP[1]);

        mcamera->LiftProjective(Eigen::Vector2d(vunNextPointsPts[i].x, vunNextPointsPts[i].y), tmpP);
        tmpP[0]  = FOCAL_LENGTH*tmpP[0]/tmpP[2] + ImageWidth/2.0;
        tmpP[1]  = FOCAL_LENGTH*tmpP[1]/tmpP[2] + ImageWidth/2.0;
        vunNextPointsPts[i] = cv::Point2f(tmpP[0], tmpP[1]);
    }

#endif
    // the solution of the fundamental matrix needs eight points
    if (mvNextPointsPts.size() > 8 && mvPrePointsPts.size() > 8)
    {
//        vector<cv::Point2f> vunNextPointsPts(mvNextPointsPts.size()), vunPrePointsPts(mvPreKeyPoints.size());
        cv::findFundamentalMat(vunPrePointsPts, vunNextPointsPts, cv::FM_RANSAC, 1.0, 0.99, status);

        DeleteErrStatus(mvPrePointsPts, status);
        DeleteErrStatus(mvNextPointsPts, status);
        DeleteErrStatus(mvCurPointsPts, status);
    }
    else
    {
        cout << "the number of the points is too little when solve the fundamental matrix" << endl;
    }

    int i = 0;
    for (auto v:status)
    {
        if (v == 0)
            i++;
    }
    cout << "F:" << status.size() << " " << i << endl;


}

} // namespace RAIN_VIO
