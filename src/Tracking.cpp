//
// Created by rain on 17-9-19.
//

#include "Tracking.h"

#include "TicToc.h"


namespace RAIN_VIO
{

Tracking::Tracking()
{
}
Tracking::Tracking(const string &strSettingsFile, int nWindowSize)
{
    mstrSettingsFile = strSettingsFile;

    EQUALIZE = true;
    mbFirstImage = true;
    mFirstImageTime = 0;
    mIDcnt = 0;
    mdFrameCount = 0;
    etrackingState = NO_INITIALIZED;
    mnWindowSize = nWindowSize;

    mmpFrames.clear();

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

    if (!fsSettings.isOpened())
    {
        LOG(FATAL) << "Failed to open settings file at " << strSettingsFile << endl;
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

    mnumFeatures = fsSettings["ORBextractor.numFeatures"];
    minDist = fsSettings["ORBextractor.minDist"];

    LOG(INFO) << "Camera Parameters: " << endl;
    LOG(INFO) << "- fx: " << CmaeraK.at<float>(0, 0) << endl;
    LOG(INFO) << "- fy: " << CmaeraK.at<float>(1, 1) << endl;
    LOG(INFO) << "- cx: " << CmaeraK.at<float>(0, 2) << endl;
    LOG(INFO) << "- cy: " << CmaeraK.at<float>(1, 2) << endl;
    LOG(INFO) << "- k1: " << DistCoef.at<float>(0) << endl;
    LOG(INFO) << "- k2: " << DistCoef.at<float>(1) << endl;
    LOG(INFO) << "- p1: " << DistCoef.at<float>(2) << endl;
    LOG(INFO) << "- p2: " << DistCoef.at<float>(3) << endl;
    LOG(INFO) << "- Image height: " << ImageHeight << endl;
    LOG(INFO) << "- Image width: " << ImageWidth << endl;

    mpMap = new Map(nWindowSize);
    mpInitializer = new Initializer(CmaeraK, mpMap, nWindowSize);
    mpCamera = new Camera(strSettingsFile);
    mpFeature = new Feature(mpCamera, strSettingsFile, mnWindowSize);
}

Tracking::~Tracking()
{
}

void Tracking::Track(const cv::Mat &image, const double &TimeStamps)
{
    vector<pair<int, Eigen::Vector3d>> Features;

    Frame CurrentFrame(mpCamera, mpFeature, mstrSettingsFile, mnWindowSize);

    CurrentFrame.DetectKeyPoint(image, TimeStamps);

    // the list of the map point
    mmpFrames.insert(make_pair(mdFrameCount, &CurrentFrame));

    // whether keyframe or not
    if (mpMap->AddFeatureCheckParallax(mdFrameCount, CurrentFrame.mvFraFeatures))
    {
        // this is a keyframe
        eMarginflag = MARGINOLD;
    }
    else
        eMarginflag = MARGINSECONDNEW;

    if (etrackingState == NO_INITIALIZED)
    {
        if (mdFrameCount == mnWindowSize)
        {
            if (InitialStructure())
            {
                etrackingState = OK;
            }
            else
            {
                SlideWindow();
            }
        }
        else
            mdFrameCount++;
    }
    else
    {
        SlideWindow();
    }
}

void Tracking::SlideWindow()
{
    if (eMarginflag == MARGINOLD) // remove the frame 0
    {
        if (mdFrameCount == mnWindowSize)
        {
            mpMap->RemoveBack();

//            mmpFrames.erase(mmpFrames.begin(), );
        }
    }
    else
    {
        if (mdFrameCount == mnWindowSize) // remove the frame N
        {
            mpMap->RemoveFront(mdFrameCount);
        }
    }
}


bool Tracking::InitialStructure()
{
    Eigen::Matrix3d RelativeR;
    Eigen::Vector3d RelativeT;
    int l = 0;

    Eigen::Quaterniond Rqwc[mdFrameCount+1];
    Eigen::Vector3d twc[mdFrameCount+1];
    map<int, Eigen::Vector3d> SFMPoint3d;

    vector<SFMFeature> vSFMFeature;

    // MapPoint
    for (auto &MapPoint : mpMap->mlMapPoints)
    {
        int idx = MapPoint.mnStartFrame - 1;

        SFMFeature tmpSFMFeature;
        tmpSFMFeature.State = false;
        tmpSFMFeature.Id = MapPoint.mnFeatureID;

        for (auto &FeaturePerFrame : MapPoint.mvFeaturePerFrame)
        {
            idx++;
            Eigen::Vector3d Point3d = FeaturePerFrame.Point;

            tmpSFMFeature.Observation.emplace_back(make_pair(idx, Eigen::Vector2d{Point3d.x(), Point3d.y()}));
        }

        vSFMFeature.emplace_back(tmpSFMFeature);
    }

    // RelativeR Rij, i == l == 0
    if (!mpInitializer->RelativePose(RelativeR, RelativeT, l))
    {
        LOG(WARNING) << "there is not enough parallax between the two frames" << endl;
        return false;
    }

    GlobalSFM GSFM;

    if (!GSFM.Construct(mdFrameCount+1, Rqwc, twc, l, RelativeR, RelativeT, vSFMFeature, SFMPoint3d))
    {
        LOG(ERROR) << "global SFM failed " << endl;
        eMarginflag = MARGINOLD;
        return false;
    }

    for (int i = 0; i < mdFrameCount; i++)
    {

    }
}

} // namespace RAIN_VIO
