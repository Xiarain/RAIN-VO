//
// Created by rain on 17-11-24.
//

#include "MapDrawer.h"

namespace RAIN_VIO
{

MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if (!fSettings.isOpened())
    {
        LOG(FATAL) << "Failed to open settings file at " << strSettingPath << endl;
        exit(-1);
    }

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

    mCameraPose.setIdentity();
}

//
/**
 * @brief use this transformation can directly to display
 * @param Twc from the camere to world
 * @return the pangolin opengl matrix type
 */
pangolin::OpenGlMatrix MapDrawer::toOpenGLMatrix(const Eigen::Matrix<double, 3, 4> Twc)
{
    pangolin::OpenGlMatrix M;

    Eigen::Matrix3d Rwc = Twc.block<3, 3>(0, 0);
    Eigen::Vector3d twc = Twc.block<3, 1>(0, 3);

    twc.data();

    M.m[0] = Rwc(0,0);
    M.m[1] = Rwc(1,0);
    M.m[2] = Rwc(2,0);
    M.m[3] = 0.0;

    M.m[4] = Rwc(0,1);
    M.m[5] = Rwc(1,1);
    M.m[6] = Rwc(2,1);
    M.m[7] = 0.0;

    M.m[8] = Rwc(0,2);
    M.m[9] = Rwc(1,2);
    M.m[10] = Rwc(2,2);
    M.m[11] = 0.0;

    M.m[12] = twc(0);
    M.m[13] = twc(1);
    M.m[14] = twc(2);
    M.m[15] = 1.0;

    return M;
}

void MapDrawer::DrawMapPoints()
{
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for (size_t i = 0; i < 10; i++)
    {
        glVertex3f(1.0, 1.0, 1.0*i);
    }

    glEnd();
}

// from the camera to world
void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize; // width
    const float h = w*0.75; // height
    const float z = w*0.6; // z axis

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame *> vpKFs = mpMap->mvpKeyFrames;

    if (bDrawKF)
    {
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            Eigen::Matrix<double, 3, 4> Twc = pKF->GetPose();

            Eigen::Matrix3d Rwc = Twc.block<3, 3>(0, 0);
            Eigen::Vector3d twc = Twc.block<3, 1>(0, 3);

            Twc.block<3, 3>(0, 0) = Rwc.inverse();
            Twc.block<3, 1>(0, 3) = -Rwc.inverse()*twc;

            pangolin::OpenGlMatrix M;

            M = toOpenGLMatrix(Twc);

            glPushMatrix();

            glMultMatrixd(M.m);

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

}

// from the wrold to camera
void MapDrawer::SetCurrentCameraPose(const Eigen::Matrix<double, 3, 4> &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw;
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    Eigen::Matrix3d Rwc;
    Eigen::Vector3d twc;

    {
        unique_lock<mutex> lock(mMutexCamera);
        Rwc = mCameraPose.block<3, 3>(0, 0).transpose();
        twc = -Rwc*mCameraPose.block<3, 1>(0, 3);
    }

    M = toOpenGLMatrix(mCameraPose);
}


} // namesapce RAIN_VIO
