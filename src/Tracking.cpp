//
// Created by rain on 17-9-19.
//

#include "Tracking.h"
#include "TicToc.h"


namespace RAIN_VIO
{

Tracking::Tracking(const string &strSettingsFile, Map *pMap,  MapDrawer *pMapDrawer): mpMap(pMap), mpMapDrawer(pMapDrawer)
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
    if (mpMap->AddFeatureCheckParallax(mpCurrentFrame, mnFrameCount, mpCurrentFrame->mvFraFeatures))
    {
        // this is a keyframe
        eMarginflag = MARGINOLD;
    }
    else
        eMarginflag = MARGINSECONDNEW;

    mpMap->GetMapPointCount();

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

        TrackReferenceKeyFrame();
        TrackReferenceKeyFrame2();
//
//        mpMap->Triangulate(&maFramesWin);
//
//        Optimizer::ComputeReprojectionCost(mnFrameCount, mpCurrentFrame ,mpMap);
//        Optimizer::ComputeReprojectionCost(5, maFramesWin.at(5), mpMap);

//        Optimizer::PoseOptimization(mnFrameCount, mpCurrentFrame ,mpMap);

//        mpMapDrawer->SetCurrentCameraPose(mpCurrentFrame->GetPose());

         SlideWindow();
    }

    cv::waitKey(0);

    if (!mpCurrentFrame->mViwerShow.empty())
        cv::imshow(" ", mpCurrentFrame->mViwerShow);

    if (!mpCurrentFrame->mViwerShow.empty())
        mpViewer->UpdateFrame(this);
} // void Tracking::Track()

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

    Eigen::Quaterniond Rqcw[mnFrameCount+1];
    Eigen::Vector3d tcw[mnFrameCount+1];
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

    if (!GSFM.Construct(mnFrameCount+1, Rqcw, tcw, l, RelativeR, RelativeT, vSFMFeature, mSFMPoint3d))
    {
        LOG(ERROR) << "global SFM failed " << endl;
        eMarginflag = MARGINOLD;
        return false;
    }

    // the set the pose of the keyframes in the slide window
    for (int i = 0; i < mnFrameCount+1; i++)
    {
        maFramesWin.at(i)->SetPose(Rqcw[i], tcw[i]);
        KeyFrame * pKF = new KeyFrame(maFramesWin.at(i), mpMap);
        mpMap->mvpKeyFrames.push_back(pKF);
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
    vector<cv::Point2f> vPoint2d;
    vector<cv::Point3f> vPoint3d;

    // TODO the MapPoint should build a reverse file
    for (auto &MapPoint : mpMap->mlMapPoints)
    {
        MapPoint.mnUsedNum = (int)MapPoint.mvFeaturePerFrame.size();

        if (!(MapPoint.mnUsedNum >= 2 && MapPoint.mnStartFrame < gWindowSize - 2))
            continue;

        if (!(MapPoint.mdEstimatedDepth > 0))
            continue;

        int MapPointID = MapPoint.mnFeatureID; // the ID comes from the feature point detection

        auto it = find_if(mpCurrentFrame->mvFraFeatures.begin(), mpCurrentFrame->mvFraFeatures.end(), [MapPointID](const pair<uint, Eigen::Vector3d> &it)
                {
                    return it.first == MapPointID;
                });

        if (it == mpCurrentFrame->mvFraFeatures.end())
        {
            continue;
        }
        else
        {
            vPoint2d.emplace_back(cv::Point2d((float)it.base()->second[0]/it.base()->second[2], (float)it.base()->second[1]/it.base()->second[2]));
        }

        vPoint3d.emplace_back(Converter::toCvPoint3f(MapPoint.mPoint3d));
    }

    LOG_IF(ERROR, vPoint3d.size() < 20) << " the number of feature is too littel: " << vPoint3d.size() << endl;;

    cv::Mat R, rvec, t, D, K;

    K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0,
                                   0, 1.0, 0,
                                   0, 0, 1.0);

    if (!cv::solvePnP(vPoint3d, vPoint2d, K, cv::Mat(), rvec, t, false))
    {
        LOG(ERROR) << " failed in solving the PnP problem " << endl;
        return false;
    }

    cv::Rodrigues(rvec, R);

    // from the camera to the world
    {
        Eigen::Matrix3d Rwc;
        Eigen::Vector3d twc;

        Rwc = Converter::toMatrix3d(R);
        twc = Converter::toVector3d(t);

        Eigen::Matrix<double, 3, 4> Twc;
        Twc.block<3, 3>(0, 0) = Rwc;
        Twc.block<3, 1>(0, 3) = twc;

        mpCurrentFrame->SetPoseInverse(Twc);
    }

    cout << mpCurrentFrame->GetTranslation().transpose() << endl;
    cout << Converter::toEuler(mpCurrentFrame->GetRotation()).transpose() << endl;

    return true;
}

bool Tracking::TrackReferenceKeyFrame2()
{
    vector<cv::Point2f> vPoint2d;
    vector<cv::Point3f> vPoint3d;

    vector<FeaturePerFrame> vFeaturePerFrame;

    vFeaturePerFrame.assign(mpCurrentFrame->mvFeaturePerFrame.begin(), mpCurrentFrame->mvFeaturePerFrame.end());

    for (auto &featurePerFrame:vFeaturePerFrame)
    {
        if (featurePerFrame.mpInvertMapPoint->mnUsedNum <= 2)
            continue;

        if (featurePerFrame.mpInvertMapPoint->mdEstimatedDepth <= 0)
            continue;

        vPoint2d.emplace_back(cv::Point2f((float)featurePerFrame.Point[0], (float)featurePerFrame.Point[1]));
        vPoint3d.emplace_back(Converter::toCvPoint3f(featurePerFrame.mpInvertMapPoint->mPoint3d));
    }

    LOG_IF(ERROR, vPoint3d.size() < 20) << " the number of feature is too littel: " << vPoint3d.size() << endl;;

    cv::Mat R, rvec, t, D, K;

    K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0,
            0, 1.0, 0,
            0, 0, 1.0);

    if (!cv::solvePnP(vPoint3d, vPoint2d, K, cv::Mat(), rvec, t, false))
    {
        LOG(ERROR) << " failed in solving the PnP problem " << endl;
        return false;
    }

    cv::Rodrigues(rvec, R);

    // from the camera to the world
    {
        Eigen::Matrix3d Rwc;
        Eigen::Vector3d twc;

        Rwc = Converter::toMatrix3d(R);
        twc = Converter::toVector3d(t);

        Eigen::Matrix<double, 3, 4> Twc;
        Twc.block<3, 3>(0, 0) = Rwc;
        Twc.block<3, 1>(0, 3) = twc;

        mpCurrentFrame->SetPoseInverse(Twc);
    }

    cout << mpCurrentFrame->GetTranslation().transpose() << endl;
    cout << Converter::toEuler(mpCurrentFrame->GetRotation()).transpose() << endl;

    return true;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer = pViewer;
}

} // namespace RAIN_VIO
