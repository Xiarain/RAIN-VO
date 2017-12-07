//
// Created by rain on 17-10-7.
//

#include "Map.h"

namespace RAIN_VIO
{

int MapPoint::EndFrame()
{
    return mnStartFrame + static_cast<int>(mvFeaturePerFrame.size()) - 1;
}


Map::Map()
{
    mlMapPoints.clear();
}
/**
 * @brief check the parallax of the frame to label the frame is the keyframe or not
 * @param frame_count the number of the frame in the slide window
 * @param Feature const vector<pair<int, Eigen::Vector3d>>, make up with the ID and keypoint
 * @return if the frame is the keyframe, return true
 */
bool Map::AddFeatureCheckParallax(const int FrameCount, const vector<pair<uint, Eigen::Vector3d>> & Features)
{
    mLastTrackNum = 0;
    double ParallaxSum = 0;
    int ParallaxNum = 0;

    for (auto Feature : Features)
    {
        FeaturePerFrame featurePerFrame(Feature.second); // the feature's position in the image
        uint featureID = Feature.first; // the ID comes from the feature point detection

        auto it = find_if(mlMapPoints.begin(), mlMapPoints.end(), [featureID](const MapPoint &it)
                    {
                        return it.mnFeatureID == featureID;
                    });

        // this MapPoint is not in the list and add the MapPoint to the list
        if (it == mlMapPoints.end())
        {
            // FrameCount is the start frame of this mappoint
            mlMapPoints.emplace_back(MapPoint(featureID, FrameCount));
            mlMapPoints.back().mvFeaturePerFrame.push_back(featurePerFrame);
        }
        else if (it->mnFeatureID == featureID)
        {
            it->mvFeaturePerFrame.push_back(featurePerFrame);

            // the number of the point from the last frame has been tracked by this frame
            mLastTrackNum++;
            // cout << "the ID: " << it->mnFeatureID << " the cnt: " <<  it->mvFeaturePerFrame.size() << endl;
        }
    } // for (auto Feature : Features)

    // TODO add the feature point to the Map and the compute the parallax should be separated, and whether the frame is the keyframe should add some other requirments
    if (FrameCount < 2 || mLastTrackNum < 20)
        return true;

    for (auto &MapPoint : mlMapPoints)
    {
        // the MapPoint should appear in this frame and the MapPoint also should be appear at least 2 frames
        if (MapPoint.mnStartFrame <= FrameCount - 2 && MapPoint.mnStartFrame + int(MapPoint.mvFeaturePerFrame.size()) - 1 >= FrameCount -1)
        {
            ParallaxSum += ComputeParallax(MapPoint, FrameCount);
            ParallaxNum++;
        }
    }

    if (ParallaxSum == 0)
    {
        return true;
    }
    else // TODO the threshold of the parallax should be read in the setting file
    {
        cout << "the parallaxsum / parallaxnum " << ParallaxSum / ParallaxNum << endl;
//        return (ParallaxSum / ParallaxNum >= (10/460));

        if (ParallaxSum/ParallaxNum >= (10.0/460))
            return true;
    }

    return false;
}

double Map::ComputeParallax(const MapPoint &mapPoint, int FrameCount)
{

    const FeaturePerFrame &framei = mapPoint.mvFeaturePerFrame[FrameCount - 2 - mapPoint.mnStartFrame];
    const FeaturePerFrame &framej = mapPoint.mvFeaturePerFrame[FrameCount - 1 - mapPoint.mnStartFrame];

    double ans = 0;

    Eigen::Vector3d pi = framei.Point;
    Eigen::Vector3d pj = framej.Point;

    double ui = pi(0) / pi(2);
    double vi = pi(1) / pi(2);
    double uj = pj(0) / pj(2);
    double vj = pj(1) / pj(2);

    double du = ui - uj;
    double dv = vi - vj;

    ans = max(ans, sqrt(du*du + dv*dv));

    return ans;
}

/**
 * @brief get the corresponding point in the slide window
 * @param FrameCount1
 * @param FrameCount2
 * @return
 */
vector<pair<Eigen::Vector3d, Eigen::Vector3d>> Map::GetCorresponding(int FrameCount1, int FrameCount2)
{
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> corres;

    for (auto &MapPoint : mlMapPoints)
    {
        if (MapPoint.mnStartFrame <= FrameCount1 && MapPoint.EndFrame() >= FrameCount2)
        {
            Eigen::Vector3d a = Eigen::Vector3d::Zero();
            Eigen::Vector3d b = Eigen::Vector3d::Zero();

            int idx1 = FrameCount1 - MapPoint.mnStartFrame;
            int idx2 = FrameCount2 - MapPoint.mnStartFrame;

            a = MapPoint.mvFeaturePerFrame[idx1].Point;
            b = MapPoint.mvFeaturePerFrame[idx2].Point;

            corres.emplace_back(make_pair(a, b));
        }
    }

    return corres;
};

/**
 * @brief the frame 0 is the oldest frame, and theframe N is the newest frame in the slide window
 */
/**
 * @brief remove the map point in the frame 0
 */
void Map::RemoveBack()
{
    cout << "map remove back" << endl;
    for (auto it = mlMapPoints.begin(), itNext = mlMapPoints.begin(); it != mlMapPoints.end(); it = itNext)
    {
        itNext++;

        if(it->mnStartFrame != 0)
            it->mnStartFrame--;
        else // the feature point start to appear in the 0 ID frame
        {
            it->mvFeaturePerFrame.erase(it->mvFeaturePerFrame.begin());

            // when the 3D feature point doesn't have the 2D feature point in the image, it will be removed in the slide window
            if (it->mvFeaturePerFrame.size() == 0)
                mlMapPoints.erase(it);
        }
    }
}

/**
 * @brief remove the frame N
 * @param FrameCount the number of the current frame
 */
void Map::RemoveFront(int FrameCount)
{
    cout << "map remove front" << endl;
    for (auto it = mlMapPoints.begin(), itNext = mlMapPoints.begin(); it != mlMapPoints.end(); it = itNext)
    {
        itNext++;

        // throw the N-1 frame, so the 3D feature point in the N frame turn to the N-1 frame
        if (it->mnStartFrame == FrameCount)
        {
            it->mnStartFrame--;
        }
        else
        {
            int j = mnWindowSize - 1 - it->mnStartFrame;

            it->mvFeaturePerFrame.erase(it->mvFeaturePerFrame.begin() + j);

            // when the 3D feature point doesn't have the 2D feature point in the image, it will be removed in the slide window
            if (it->mvFeaturePerFrame.size() == 0)
                mlMapPoints.erase(it);
        }
    }
}

void Map::DebugShow()
{
    LOG(INFO) << "the map debug: show the map point in the map" << endl;
    LOG(INFO) << "the size of the map points " << static_cast<int>(mlMapPoints.size()) << endl;
    LOG(INFO) << "the ID: the number of the used: the start frame" << endl;
    for (auto point : mlMapPoints)
    {
        LOG(INFO) <<  point.mnFeatureID << " " << static_cast<int>(point.mvFeaturePerFrame.size()) << " " << point.mnStartFrame << endl;
    }
}

/**
 * triangulate and estimated the depth in the all keyframe slide window
 * @param paFramesWin
 */
void Map::Triangulate(array<Frame *, (gWindowSize+1)> *paFramesWin)
{
    // TODO mnWindowSize should be a global variable
    for (auto &MapPoint : mlMapPoints)
    {
        MapPoint.mnUsedNum = (int)MapPoint.mvFeaturePerFrame.size();

        if (!(MapPoint.mnUsedNum >= 2 && MapPoint.mnStartFrame < gWindowSize-2))
            continue;

        if (MapPoint.mdEstimatedDepth > 0)
            continue;

        int Frame0num = MapPoint.mnStartFrame;
        int Frame1num = Frame0num - 1;

        Eigen::MatrixXd SVDA(2*MapPoint.mvFeaturePerFrame.size(), 4);

        Eigen::Matrix3d Rc0w = paFramesWin->at((size_t)Frame0num)->GetRotation();
        Eigen::Vector3d tc0w = paFramesWin->at((size_t)Frame0num)->GetTranslation();

        LOG_IF(WARNING, tc0w[0] == 0 & tc0w[1] == 0 & tc0w[2] == 0) << "the Pose0 of the camera is wrong" << endl;

        int SVDIdx = 0;

        // the observation 2D feature point of the MapPoint in per frame
        for (auto &itFeaturePerFrame : MapPoint.mvFeaturePerFrame)
        {
            Frame1num++;

            Eigen::Matrix3d Rc1w = paFramesWin->at((size_t)Frame1num)->GetRotation();
            Eigen::Vector3d tc1w = paFramesWin->at((size_t)Frame1num)->GetTranslation();

            LOG_IF(WARNING, tc1w[0] == 0 & tc1w[1] == 0 & tc1w[2] == 0) << "the Pose1 of the camera is wrong"<< endl;

            Eigen::Matrix3d Rc1c0 = Rc0w.transpose()*Rc1w;
            Eigen::Vector3d tc1c0 = Rc0w.transpose()*(tc1w - tc0w);

            Eigen::Matrix<double, 3, 4> Tc1c0;
            Tc1c0.block<3, 3>(0, 0) = Rc1c0;
            Tc1c0.block<3, 1>(0, 3) = tc1c0;

            Eigen::Vector3d Point2d = itFeaturePerFrame.Point.normalized();
            SVDA.row(SVDIdx++) = Point2d[0]*Tc1c0.row(2) - Point2d[2]*Tc1c0.row(0);
            SVDA.row(SVDIdx++) = Point2d[1]*Tc1c0.row(2) - Point2d[2]*Tc1c0.row(1);

            // TODO: think over
            if (Frame1num == Frame0num)
                continue;
        }

        CHECK(SVDIdx == SVDA.rows()) << "the svd matrix construction wrong";

        Eigen::Vector4d SVDV = Eigen::JacobiSVD<Eigen::MatrixXd>(SVDA, Eigen::ComputeThinV).matrixV().rightCols<1>();

        MapPoint.mdEstimatedDepth = SVDV[2]/SVDV[3];

        LOG_IF(INFO, MapPoint.mdEstimatedDepth < 0.1) << "the estimated depth of the MapPoint is too small " << MapPoint.mdEstimatedDepth << endl;
        LOG_IF(INFO, MapPoint.mdEstimatedDepth > 10) << "the estimated depth of the MapPoint is too big " << MapPoint.mdEstimatedDepth << endl;

        if (MapPoint.mdEstimatedDepth < 0.1)
        {
            MapPoint.mdEstimatedDepth = 5.0;
        }

        MapPoint.mPoint3d = MapPoint.mdEstimatedDepth*MapPoint.mvFeaturePerFrame[Frame0num].Point;
    } // for (auto &MapPoint : mlMapPoints)
}

vector<KeyFrame *> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame *>(mvpKeyFrames.begin(), mvpKeyFrames.end());
}

}// namesapce RAIN_VIO