//
// Created by rain on 17-9-17.
//


#include <fstream>
#include "System.h"
#include "Initializer.h"
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

void LoadImagesName(const string &strImagePath, const string &strTimesPath, vector<string> &vstrImages, vector<double> &vTimeStamps);

void ReadParameter();


// ./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml
// PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt
int main(int argc,char* argv[])
{
    google::InitGoogleLogging(argv[0]);
//    google::SetStderrLogging(google::INFO);
    FLAGS_colorlogtostderr = true;
    FLAGS_log_dir = "../log";
    LOG(INFO) << "RAIN VO monocular vision odometry";

    // In order to debug the code in the convenient way, the settings file path is writen in here
    string strSettingsFile("../config/EuRoC/EuRoC.yaml");
    string strImagePath("../DataSheet/mav0/cam0/data");
    string strTimesPath("../DataSheet/EuRoC_TimeStamps/MH03.txt");

    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImagesName(strImagePath, strTimesPath, vstrImageFilenames, vTimestamps);

    RAIN_VIO::System VIO(strSettingsFile, true);

    int numImages = vstrImageFilenames.size();

    if (numImages <= 0)
    {
        LOG(FATAL) << "Failed to load images sequences";
        return 1;
    }

    cv::Mat image;
    for (int i = 0; i < numImages; i++)
    {
        image = cv::imread(vstrImageFilenames[i], CV_LOAD_IMAGE_UNCHANGED);

        if(image.empty())
        {
            LOG(FATAL) << "Failed to load images at " << vstrImageFilenames[i];
            return 1;
        }

        VIO.TrackMono(strSettingsFile, image, vTimestamps[i]);
    }

    google::ShutdownGoogleLogging();

    return 0;
}

/**
 * @brief read the image file in the datasheet
 * @param strImagePath image files path
 * @param strPathTimes the time stamp of the image file
 * @param vstrImages output the name of the images
 * @param vTimeStamps output the time stamp of the image file
 */
void LoadImagesName(const string &strImagePath, const string &strTimesPath, vector<string> &vstrImages, vector<double> &vTimeStamps)
{

    // c++ library, the file stream input
    ifstream fTimes;
    fTimes.open(strTimesPath.c_str());

    if (!fTimes.is_open())
    {
        LOG(FATAL) << "Failed to open time stamp file at " << strTimesPath << endl;
        exit(-1);
    }

    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

