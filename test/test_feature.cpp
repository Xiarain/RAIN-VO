//
// Created by rain on 17-12-11.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "TicToc.h"

using namespace std;

int main ( int argc, char** argv )
{

    cv::Mat img1 = cv::imread("../test/test_pnp/1.png", CV_LOAD_IMAGE_COLOR );
    if (img1.empty())
    {
        cout << "can not load the image " << endl;
        return 0;
    }
    cv::Mat img2 = cv::imread("../test/test_pnp/2.png", CV_LOAD_IMAGE_COLOR );

    RAIN_VIO::TicTOC tictoc;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create( "BruteForce-Hamming");


    detector->detect(img1,keypoints_1);
    detector->detect(img2,keypoints_2);


    descriptor->compute(img1, keypoints_1, descriptors_1);
    descriptor->compute(img2, keypoints_2, descriptors_2);

//    Mat outimg1;
//    drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//    imshow("ORB feature",outimg1);

    vector<cv::DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);

    double min_dist=10000, max_dist=0;

    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

//    min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
//    max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector<cv::DMatch> good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    cv::Mat img_match;
    cv::Mat img_goodmatch;

    cout << "matches: " << matches.size() << endl;
    cout << "good matche: " << good_matches.size() << endl;
    cout << "times(ms): " << tictoc.toc() << endl;

//    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    cv::drawMatches ( img1, keypoints_1, img2, keypoints_2, good_matches, img_goodmatch );
//    imshow ( "所有匹配点对", img_match );
    cv::imshow ( "good match", img_goodmatch );
    cv::waitKey(0);

    return 0;
}