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
 * @brief
 * @param frame_count the number of the frame
 * @param Feature const vector<pair<int, Eigen::Vector3d>>, make up with the ID and keypoint
 * @return
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
            mLastTrackNum++; // the number of the point from the last frame has been tracked by this frame
//            cout << "the ID: " << it->mnFeatureID << " the cnt: " <<  it->mvFeaturePerFrame.size() << endl;
        }
    } // for (auto Feature : Features)


    if (FrameCount < 2 || mLastTrackNum < 20)
        return true;

    for (auto &MapPoint : mlMapPoints)
    {
        // the MapPoint should appear in this frame and the MapPoint also should be appear at least 2 frames
        if (MapPoint.mnStartFrame <= FrameCount -2 && MapPoint.mnStartFrame + int(MapPoint.mvFeaturePerFrame.size()) - 1 >= FrameCount -1)
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
        return (ParallaxSum / ParallaxNum >= 10);
    }
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

}// namesapce RAIN_VIO