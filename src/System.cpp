//
// Created by rain on 17-9-18.
//

#include "System.h"

#include <opencv2/opencv.hpp>


using namespace std;

namespace RAIN_VIO
{

System::System(const string &strSettingsFile, const bool bOpenViwer)
{
    mpTracker = new Tracking(strSettingsFile);
    mpViewer = new Viewer(strSettingsFile);

    mptViewer = new thread(&Viewer::Run, mpViewer);

    mpTracker->SetViewer(mpViewer);
}

/**
 * @brief track
 * @param image raw image from the camera or the datasheet
 * @param TimeStamps the time stamps of this frame
 */
void System::TrackMono(const string &strSettingsFile, const cv::Mat &image, const double &TimeStamps)
{
//    cv::imshow("Raw image", image);
//    cv::waitKey(5);
    mpTracker->Track(image, TimeStamps);

}



} // namespace RAIN_VIO
