//
// Created by rain on 17-10-21.
//

#include "Viewer.h"
#include <pangolin/pangolin.h>

namespace RAIN_VIO
{

Viewer::Viewer(const string &strSettingPath)
{
    cv::FileStorage fsSettings(strSettingPath.c_str(), cv::FileStorage::READ);

    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at " << strSettingPath << endl;
        exit(-1);
    }
}

void Viewer::Run()
{
    pangolin::CreateWindowAndBind("RAIN_VIO: Map Viewer", 1024, 768);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.7,1.0,0.0,pangolin::Attach::Pix(175));

    // the name of the buttion; default setting; whether the selection box;
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
            pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0)
    );

    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    while(1)
    {
        s_cam.Follow(Twc);

        d_cam.Activate(s_cam);

        glClearColor(1.0f,1.0f,1.0f,1.0f);

        pangolin::FinishFrame();
    }
}

} // namespace RAIN_VIO
