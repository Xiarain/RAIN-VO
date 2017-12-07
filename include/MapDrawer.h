//
// Created by rain on 17-11-24.
//

#ifndef RAIN_VIO_MAPDRAWER_H
#define RAIN_VIO_MAPDRAWER_H

#include <eigen3/Eigen/Dense>
#include "Map.h"
#include "KeyFrame.h"


namespace RAIN_VIO
{

class Map;
class KeyFrame;

class MapDrawer
{
public:

    MapDrawer(Map* pMap, const string &strSettingPath);

    pangolin::OpenGlMatrix toOpenGLMatrix(const Eigen::Matrix<double, 3, 4> T);

    void DrawMapPoints();

    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

    void SetCurrentCameraPose(const Eigen::Matrix<double, 3, 4> &Twc);

//    void SetReferenceKeyFrame(KeyFrame *pKF);

    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &Mcw);

    Map *mpMap;

private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    Eigen::Matrix<double, 3, 4> mCameraPose;

    std::mutex mMutexCamera;

}; // class MapDrawer

} // namespace RAIN_VIO

#endif //RAIN_VIO_MAPDRAWER_H
