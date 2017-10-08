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
            mlMapPoints.push_back(MapPoint(featureID, FrameCount)); // FrameCount is the start frame of this mappoint
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
}

}// namesapce RAIN_VIO