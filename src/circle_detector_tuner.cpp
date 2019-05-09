//
// Created by eirik on 09.05.19.
//

/**
 * @file HoughCircle_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

// windows and trackbars name
const std::string windowName = "Hough Circle Detection Demo";
const std::string cannyThresholdTrackbarName = "Canny threshold";
const std::string accumulatorThresholdTrackbarName = "Accumulator Threshold";
const std::string usage = "Usage : tutorial_HoughCircle_Demo <path_to_input_image>\n";

// initial and max values of the parameters of interests.
const int cannyThresholdInitialValue = 100;
const int accumulatorThresholdInitialValue = 50;
const int maxAccumulatorThreshold = 200;
const int maxCannyThreshold = 255;
const int blurKernelSize = 5;

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

void writeHoughParams(Rect2d r, int cannyThreshold, int accumulatorThreshold, const char* filePath)
{
    FileStorage fs(filePath, FileStorage::WRITE);
    fs << "cropRegion" << r;
    fs << "blurKernelSize" << blurKernelSize;
    fs << "cannyThreshold" << cannyThreshold;
    fs << "accumulatorThreshold" << accumulatorThreshold;

    fs.release();
}


int main(int argc, char** argv)
{
    Mat src, src_gray, imCrop;

    // Read the image
    String imageName("../data/stuff.jpg"); // by default
    if (argc > 1)
    {
        imageName = argv[1];
    }
    src = imread( imageName, IMREAD_COLOR );

    if( src.empty() )
    {
        std::cerr<<"Invalid input image\n";
        std::cout<<usage;
        return -1;
    }


    // Crop image
    Rect2d r = selectROI(src);
    imCrop = src(r);
    src = imCrop;

    // Convert it to gray
    cvtColor(imCrop, src_gray, COLOR_BGR2GRAY );

    // Reduce the noise so we avoid false circle detection
    GaussianBlur(src_gray, src_gray, Size(blurKernelSize, blurKernelSize), 2, 2 );

    //declare and initialize both parameters that are subjects to change
    int cannyThreshold = cannyThresholdInitialValue;
    int accumulatorThreshold = accumulatorThresholdInitialValue;

    // create the main window, and attach the trackbars
    namedWindow( windowName, WINDOW_NORMAL);
    createTrackbar(cannyThresholdTrackbarName, windowName, &cannyThreshold,maxCannyThreshold);
    createTrackbar(accumulatorThresholdTrackbarName, windowName, &accumulatorThreshold, maxAccumulatorThreshold);


    // infinite loop to display
    // and refresh the content of the output image
    // until the user presses q or Q
    char key = 0;
    while(key != 'q' && key != 'Q')
    {
        // those parameters cannot be =0
        // so we must check here
        cannyThreshold = std::max(cannyThreshold, 1);
        accumulatorThreshold = std::max(accumulatorThreshold, 1);

        //runs the detection, and update the display
        HoughDetection(src_gray, src, cannyThreshold, accumulatorThreshold);

        // get user key
        key = (char)waitKey(10);
    }

    cout << "blurKernelSize: " << blurKernelSize << endl << endl;
    cout << "cannyThreshold: " <<  cannyThreshold << endl << endl;
    cout << "accumulatorThreshold: " <<  accumulatorThreshold << endl << endl;
    cout << "Crop region: " << r << endl << endl;

    writeHoughParams(r,cannyThreshold,accumulatorThreshold, argv[2]);

    return 0;
}