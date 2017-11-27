//
// Created by rain on 17-10-21.
//

#include "Viewer.h"


namespace RAIN_VIO
{

Viewer::Viewer(const string &strSettingPath, MapDrawer* pMapDrawer):mpMapDrawer(pMapDrawer)
{
    cv::FileStorage fsSettings(strSettingPath.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at " << strSettingPath << endl;
        exit(-1);
    }

    mImageHeight = fsSettings["Camera.height"];
    mImageWidth = fsSettings["Camera.width"];

    mViewpointX = fsSettings["Viewer.ViewpointX"];
    mViewpointY = fsSettings["Viewer.ViewpointY"];
    mViewpointZ = fsSettings["Viewer.ViewpointZ"];
    mViewpointF = fsSettings["Viewer.ViewpointF"];
}

void Viewer::UpdateFrame(Tracking *pTracking)
{
    unique_lock<mutex> lock(mMutex);
//    pTracking->mRawImage.copyTo(mframe);
//    mvFraPointsID = pTracking->mpCurrentFrame->mvFraPointsID;
//    mvFraPointsCnt = pTracking->mpCurrentFrame->mvFraPointsCnt;

    mframe = pTracking->mpCurrentFrame->mViwerShow;
    if (mframe.empty())
    {
        LOG(FATAL) << "the image is empty" << endl;
    }

    if (mframe.channels() == 1)
        cv::cvtColor(mframe, mframe, CV_GRAY2RGB);


}

void Viewer::Run()
{
    pangolin::CreateWindowAndBind("RAIN_VIO: Map Viewer", 1440, 900); // 1024 768

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.8,1.0,0.0,0.10);

    // the name of the buttion; default setting; whether the selection box;
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1440,900,500,500,512,389,0.1,1000),
            pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0)
    );

    pangolin::Handler3D handler(s_cam);

    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.2, 1.0, -1440.0f/900.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::View& rgbimage = pangolin::Display("image")
            .SetBounds(0.60,0.9,0.55,1.0,(-1.0)*mImageWidth/mImageHeight)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::GlTexture imageTexture(mImageWidth,mImageHeight,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    while(1)
    {


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        s_cam.Follow(Twc);

        d_cam.Activate(s_cam);

        mpMapDrawer->DrawCurrentCamera(Twc);
//        mpMapDrawer->DrawKeyFrames(true, true);
//        mpMapDrawer->DrawMapPoints();

        glClearColor(1.0f,1.0f,1.0f,1.0f);

        // the mutex lock is very important, if not, the image sometimes is blurred and program easily shutdown
        if (!mframe.empty())
        {
            unique_lock<mutex> lock(mMutex);
            imageTexture.Upload(mframe.data,GL_RGB,GL_UNSIGNED_BYTE);
        }

        //display the image
        rgbimage.Activate();
        glColor3f(1.0,1.0,1.0);

        imageTexture.RenderToViewportFlipY();

        pangolin::FinishFrame();

    } // while(1)

} // Viewer::Run()

} // namespace RAIN_VIO
