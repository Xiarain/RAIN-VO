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

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
}

} // namespace RAIN_VIO
