//
// Created by rain on 17-11-23.
//

#include "pangolin/pangolin.h"
#include <opencv2/highgui/highgui.hpp>

int main()
{
    pangolin::CreateWindowAndBind("Plane_Slam: Map Viewer",1024,768);

    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.7,1.0,0.0,0.2);

    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);


    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
            pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0)
    );

    pangolin::Handler3D handler(s_cam);

    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.2, 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    cv::Mat rgb;

    rgb = cv::imread("/home/rain/workspace/RAIN-VIO/DataSheet/mav0/cam0/data/1403637175488318976.png", CV_LOAD_IMAGE_COLOR);

    //定义图片面板
    pangolin::View& rgb_image = pangolin::Display("rgb")
            .SetBounds(0.0,0.4,0.0,0.4,(-1.0)*rgb.cols/rgb.rows)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    //初始化
    pangolin::GlTexture imageTexture(rgb.cols,rgb.rows,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

    bool Follow = true;

    int count = 0;
    while(1)
    {
        count++;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);

        glColor3f(1.0,1.0,1.0);
        pangolin::glDrawColouredCube();

        // d_cam.Activate(s_cam);
        // set white color in the backdrop
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        //_FrameDrawer->DrawFrame(rgb);
        imageTexture.Upload(rgb.data,GL_RGB,GL_UNSIGNED_BYTE);

        //display the image
        rgb_image.Activate();
        glColor3f(1.0,1.0,1.0);

        imageTexture.RenderToViewportFlipY();

        pangolin::FinishFrame();
    }

}