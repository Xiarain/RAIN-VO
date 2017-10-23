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

    mlpFrames.clear();

    mpcamera = new Camera(strSettingsFile);

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

    mnumFeatures = fsSettings["ORBextractor.numFeatures"];
    minDist = fsSettings["ORBextractor.minDist"];

    mCurrentFrame = new Frame(mstrSettingsFile, nWindowSize);
    mpMap = new Map(nWindowSize);
    mpinitializer = new Initializer(CmaeraK, mpMap, nWindowSize);
}

Tracking::~Tracking()
{
}

void Tracking::Track(const cv::Mat &image, const double &TimeStamps)
{
    vector<pair<int, Eigen::Vector3d>> Features;

    mCurrentFrame->DetectKeyPoint(image, TimeStamps);

//    if (mlpFrames.size() > 1)
//    {
//        cv::drawMatches(mlpFrames.back()->mImageShow, mlpFrames.back()->mvFraPointsPts, mCurrentFrame->mImageShow, mCurrentFrame->mvFraPointsPts, );
//    }

    // the list of the map point
    mlpFrames.emplace_back(mCurrentFrame);

    // whether keyframe or not
    if (mpMap->AddFeatureCheckParallax(mdFrameCount, mCurrentFrame->mvFraFeatures))
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

    // checke if there are enough correnspondences
}

void Tracking::SlideWindow()
{
    if (eMarginflag == MARGINOLD) // remove the frame 0
    {
        if (mdFrameCount == mnWindowSize)
        {
            mpMap->RemoveBack();
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

    Eigen::Quaterniond Q[mdFrameCount+1];
    Eigen::Vector3d T[mdFrameCount+1];
    map<int, Eigen::Vector3d> SFMPoint3d;

    vector<SFMFeature> vSFMFeature;

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

    if (!mpinitializer->RelativePose(RelativeR, RelativeT, l))
    {
        cout << "there is not enough parallax between the two frames" << endl;
        return false;
    }
    cout << RelativeR << endl;
    cout << RelativeT << endl;

    GlobalSFM GSFM;


    if (!GSFM.Construct(mdFrameCount+1, Q, T, l, RelativeR, RelativeT, vSFMFeature, SFMPoint3d))
    {
        cout << "global SFM failed " << endl;
        eMarginflag = MARGINOLD;
        return false;
    }






}

} // namespace RAIN_VIO
