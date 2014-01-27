#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"


int main( int argc, char** argv )
{
    if( argc != 3 )
    {
        std::cout << " Usage: " << argv[0] << "  <img1> <img2>" << std::endl;
        return 0;
    }

    cv::Mat img_1 = cv::imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
    cv::Mat img_2 = cv::imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );

    if( !img_1.data || !img_2.data )
    {
        std::cout<< " Error reading images " << std::endl;
        return -1;
    }

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    cv::SurfFeatureDetector detector( minHessian );
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    detector.detect( img_1, keypoints_1 );
    detector.detect( img_2, keypoints_2 );

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor extractor;
    cv::Mat descriptors_1, descriptors_2;
			
    extractor.compute( img_1, keypoints_1, descriptors_1 );
    extractor.compute( img_2, keypoints_2, descriptors_2 );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    //-- Get sets of points from image left and right
    std::vector<cv::Point2f> points1, points2;
    std::vector< cv::DMatch > good_matches;
    for( int i = 0; i < descriptors_1.rows; i++ )
    {
        if( matches[i].distance <= std::max(2*min_dist, 0.02) )
        {
            good_matches.push_back(matches[i]);
            points1.push_back(keypoints_1[matches[i].trainIdx].pt);
            points2.push_back(keypoints_2[matches[i].queryIdx].pt);
        }
    }

    //-- Fundamental Matrix  x'Fx = 0
    //-- Where x = point in image 1 and x' = match in image 2 (homogenous coordinates)
    cv::Mat fundamental = cv::findFundamentalMat(points1, points2);

    //-- Intrinsic parameters from the camera
    cv::Mat intrinsics(3, 3, CV_64FC1, cv::Scalar(0));
    intrinsics.at<double>(0, 2) = 200;
    intrinsics.at<double>(1, 2) = 355;
    intrinsics.at<double>(0, 0) = 525;
    intrinsics.at<double>(1, 1) = 525;
    intrinsics.at<double>(2, 2) = 1;

    //-- Essential matrix  y'Ey = 0
    //-- y, y' normalized image coordinates (see wikipedia)
    cv::Mat essential = intrinsics.t() * fundamental * intrinsics;

    //-- SVD decomposition
    //-- gives U, Vt and E matrices (no idea dafuq they correspond to)
    cv::Mat u, e_diag, vt;
    cv::SVD svd;
    svd.compute(essential, e_diag, u, vt);
    cv::Mat e(3, 3, CV_64FC1, cv::Scalar(0));
    e.at<double>(0, 0) = e_diag.at<double>(0, 0);
    e.at<double>(1, 1) = e_diag.at<double>(1, 0);

    //-- Builds a W matrix (Wt = W-1)
    //     0 -1  0
    // W = 1  0  0
    //     0  0  1
    cv::Mat w(3, 3, CV_64FC1, cv::Scalar(0));
    w.at<double>(1, 0) = 1;
    w.at<double>(0, 1) = -1;
    w.at<double>(2, 2) = 1;

    //-- Compute rotation matrix R
    //-- R = U Wt Vt
    cv::Mat rotation = u * w.t() * vt;

    //-- Compute matrix [t]x
    //-- [t]x = v * Z * Vt
    // with
    //     0 -1  0
    // Z = 1  0  0
    //     0  0  0
    cv::Mat z = w;
    z.at<double>(2, 2) = 0;
    cv::Mat tx = vt.t() * z * sigma * vt;
    cv::Mat translation(3, 1, CV_64FC1, cv::Scalar(0));

    //-- Compute Vector T
    //         0  -tz  ty
    // [t]x = tz  0  -tx
    //       -ty  tx  0
    // T = tx ty tz
    translation.at<double>(0, 0) = tx.at<double>(2, 1);
    translation.at<double>(1, 0) = tx.at<double>(0, 2);
    translation.at<double>(2, 0) = tx.at<double>(1, 0);

    std::cout << "Rotation matrix" << std::endl;
    std::cout << rotation << std::endl;
    std::cout << "Translation vector" << std::endl;
    std::cout << translation << std::endl;

    //-- Draw matches
    Mat img_matches;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    imshow( "Matches", img_matches );

    waitKey(0);
    
    return 0;
}