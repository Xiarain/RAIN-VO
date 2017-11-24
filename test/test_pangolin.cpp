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

    cv::Mat image;

    image = cv::imread("../DataSheet/mav0/cam0/data/1403637175488318976.png", CV_LOAD_IMAGE_COLOR);
    if(image.empty())
    {
        std::cerr << "Failed to load images, you should add the datasheet or image to the ../DataSheet/mav0/cam0/data/1403637175488318976.png" << std::endl;
        return 1;
    }
    
    pangolin::View& rgbimage = pangolin::Display("image")
            .SetBounds(0.7,1.0,0.6,1.0,(-1.0)*image.cols/image.rows)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);
    
    pangolin::GlTexture imageTexture(image.cols,image.rows,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);

        glColor3f(1.0,1.0,1.0);
        pangolin::glDrawColouredCube();

        // d_cam.Activate(s_cam);
        // set white color in the backdrop
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        //_FrameDrawer->DrawFrame(rgb);
        imageTexture.Upload(image.data,GL_RGB,GL_UNSIGNED_BYTE);

        //display the image
        rgbimage.Activate();
        glColor3f(1.0,1.0,1.0);

        imageTexture.RenderToViewportFlipY();

        pangolin::FinishFrame();
    }

}