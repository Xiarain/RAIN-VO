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


Map::Map(int nWindowSize)
{
    mlMapPoints.clear();

    mnWindowSize = nWindowSize;
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
        uint featureID = Feature.first; // the ID

        auto it = find_if(mlMapPoints.begin(), mlMapPoints.end(), [featureID](const MapPoint &it)
                    {
                        return it.mnFeatureID == featureID;
                    });

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
    cout << "the map debug: show the map point in the map" << endl;
    cout << "the size of the map points" << static_cast<int>(mlMapPoints.size()) << endl;

    cout << "the ID: the number of the used: the start frame" << endl;
    for (auto point : mlMapPoints)
    {
        cout <<  point.mnFeatureID << " " << static_cast<int>(point.mvFeaturePerFrame.size()) << " " << point.mnStartFrame << endl;
    }
}

}// namesapce RAIN_VIO