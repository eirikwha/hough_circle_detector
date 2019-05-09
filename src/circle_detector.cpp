//
// Created by eirik on 09.05.19.
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

const std::string windowName = "Hough Circle Detection Demo";

void readHoughParams(Rect2d &r, int &blurKernelSize, int &cannyThreshold, int &accumulatorThreshold, const char* filePath){

    FileStorage fs(filePath, FileStorage::READ);

    fs["cropRegion"] >> r;
    fs["blurKernelSize"] >> blurKernelSize;
    fs["cannyThreshold"] >> cannyThreshold;
    fs["accumulatorThreshold"] >> accumulatorThreshold;
    fs.release();
}

bool foundCircles(vector<Vec3f> &circles, int numCircles){

    if (circles.size() < numCircles){
        return false;
    }
}

void HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold)
{
    // will hold the results of the detection
    std::vector<Vec3f> circles;
    // runs the actual detection
    HoughCircles( src_gray, circles, HOUGH_GRADIENT, 1, src_gray.rows/8, cannyThreshold, accumulatorThreshold, 0, 0 );

    // clone the colour, input image for displaying purposes
    Mat display = src_display.clone();
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }

    // shows the results
    imshow( windowName, display);
}

/** @function main */
int main(int argc, char** argv)
{
    Rect2d r;
    int blurKernelSize, cannyThreshold, accumulatorThreshold;

    /// Read the settings
    if (argc == 3) {
        readHoughParams(r, blurKernelSize, cannyThreshold, accumulatorThreshold, argv[2]);
    }
    else{
        blurKernelSize = 5;
        cannyThreshold = 100;
        accumulatorThreshold = 50;
    }

    Mat src, src_gray, imCrop;

    TickMeter tm;
    tm.start();
    /// Read the image
    src = imread( argv[1], 1 );

    if( !src.data )
    { return -1; }

    if (argc == 3) {
        /// Crop image
        imCrop = src(r);
        src = imCrop;
    }

    /// Convert it to gray
    cvtColor( src, src_gray, CV_BGR2GRAY );

    /// Reduce the noise so we avoid false circle detection
    GaussianBlur( src_gray, src_gray, Size(blurKernelSize, blurKernelSize), 2, 2 );

    vector<Vec3f> circles;

    // TODO: Implement for two regions

    /// Apply the Hough Transform to find the circles
    HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, cannyThreshold, accumulatorThreshold, 0, 0 );

    if (foundCircles(circles, 1)){
        cout << "Part has correct orientation" << endl;
    }
    else{
        cout << "Flip part 180 degrees before placing." << endl;
    }

    tm.stop();
    std::cout << tm.getTimeSec() << endl;

    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }

    /// Show your results
    namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
    imshow( "Hough Circle Transform Demo", src );

    waitKey(0);
    return 0;
}