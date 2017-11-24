//
// Created by rain on 17-9-19.
//

#include "Tracking.h"
#include "TicToc.h"


namespace RAIN_VIO
{

Tracking::Tracking(const string &strSettingsFile)
{
    mstrSettingsFile = strSettingsFile;

    EQUALIZE = true;
    mbFirstImage = true;
    mFirstImageTime = 0;
    mIDcnt = 0;
    mnFrameCount = 0;
    etrackingState = NOINITIALIZED;

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

    mpMap = new Map(mnWindowSize);
    mpInitializer = new Initializer(CmaeraK, mpMap, mnWindowSize);
    mpCamera = new Camera(strSettingsFile);
    mpFeature = new Feature(mpCamera, strSettingsFile, mnWindowSize);
}

Tracking::~Tracking()
{
}

void Tracking::Track(const cv::Mat &image, const double &TimeStamps)
{
    vector<pair<int, Eigen::Vector3d>> Features;

    mpCurrentFrame = new Frame(mpCamera, mpFeature, mstrSettingsFile, mnWindowSize);

    mRawImage = image.clone();

    mpCurrentFrame->DetectKeyPoint(image, TimeStamps);

    // the list of the map point
    mmpFrames.insert(make_pair(mpCurrentFrame->GetFrameID(), mpCurrentFrame));

    maFramesWin.at(mnFrameCount) = mpCurrentFrame;

    // whether keyframe or not
    if (mpMap->AddFeatureCheckParallax(mnFrameCount, mpCurrentFrame->mvFraFeatures))
    {
        // this is a keyframe
        eMarginflag = MARGINOLD;
    }
    else
        eMarginflag = MARGINSECONDNEW;

    if (etrackingState == NOINITIALIZED)
    {
        if (mnFrameCount == mnWindowSize)
        {
            if (InitialStructure())
            {
                etrackingState = OK;
            }

            SlideWindow();
        }
        else
            mnFrameCount++;
    }
    else
    {
//        mpMap->Triangulate(&maFramesWin);

//        Optimizer::PoseOptimization(8, maFramesWin.at(8) ,mpMap);
//
        SlideWindow();
    }

    if (!mpCurrentFrame->mViwerShow.empty())
        mpViewer->UpdateFrame(this);
}

void Tracking::SlideWindow()
{
    if (eMarginflag == MARGINOLD) // remove the frame 0
    {
        if (mnFrameCount == mnWindowSize)
        {
            // move the number 0 array to the number N array
            for (int i = 0; i < mnWindowSize; i++)
            {
                swap(maFramesWin.at(i), maFramesWin.at(i+1));
            }

            maFramesWin.at(mnWindowSize) = maFramesWin.at(mnWindowSize-1);

//            size_t Frame0ID = maFramesWin.at(0)->GetFrameID();
//
//            map<size_t, Frame>::iterator it0;
//
//            it0 = mmpFrames.find(Frame0ID);

//            mmpFrames.erase(mmpFrames.begin(), it0);

            mpMap->RemoveBack();
        }
    }
    else
    {
        if (mnFrameCount == mnWindowSize) // remove the frame N-1
        {
            swap(maFramesWin.at(mnWindowSize-1), maFramesWin.at(mnWindowSize));

            mpMap->RemoveFront(mnFrameCount);
        }
    }
}


bool Tracking::InitialStructure()
{
    Eigen::Matrix3d RelativeR;
    Eigen::Vector3d RelativeT;
    int l = 0;

    Eigen::Quaterniond Rqwc[mnFrameCount+1];
    Eigen::Vector3d twc[mnFrameCount+1];
    map<int, Eigen::Vector3d> mSFMPoint3d;

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
        return false;
    }

    GlobalSFM GSFM;

    if (!GSFM.Construct(mnFrameCount+1, Rqwc, twc, l, RelativeR, RelativeT, vSFMFeature, mSFMPoint3d))
    {
        LOG(ERROR) << "global SFM failed " << endl;
        eMarginflag = MARGINOLD;
        return false;
    }

    // the set the pose of the keyframes in the slide window
    for (int i = 0; i < mnFrameCount+1; i++)
    {
        maFramesWin.at(i)->SetPose(Rqwc[i], twc[i]);
    }

    for (auto &MapPoint : mpMap->mlMapPoints)
    {
        auto it = mSFMPoint3d.find(MapPoint.mnFeatureID);

        if (it == mSFMPoint3d.end())
        {
            continue;

        } else{

            MapPoint.mPoint3d = it->second;
            MapPoint.mdEstimatedDepth = it->second[2];
            // cout << MapPoint.mnFeatureID << " " << MapPoint.mPoint3d.transpose() << endl;
        }
    }

    // TODO there are some keyframes needed to solve the position
}

bool Tracking::TrackReferenceKeyFrame()
{

}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer = pViewer;
}

} // namespace RAIN_VIO
