//
// Created by eirik on 02.06.19.
//

#include "hough_circle_detector/circle_detector_class.h"

using namespace cv;
using namespace std;

// TODO: Implement some channel processing??
//  https://stackoverflow.com/questions/9860667/writing-robust-color-
//  and-size-invariant-circle-detection-with-opencv-based-on

HoughCircleDetector::HoughCircleDetector(){
    setDefaultHoughParams();
    minNumCircles = 1; // TODO: should parse this some way
}

HoughCircleDetector::HoughCircleDetector(std::string detectorParamPath): paramFilePath(detectorParamPath){
    readHoughParams();
    minNumCircles = 1;
}

HoughCircleDetector::~HoughCircleDetector(){};

cv::Mat HoughCircleDetector::getDisplayImage(){
    return imgDisplay;
}

bool HoughCircleDetector::getDetectionResult(cv::Mat &img){
    img = imgSource;
    detectCircles(imgSource);
    if (circles.size() < minNumCircles){
        return false;
    }
}

void HoughCircleDetector::setDefaultHoughParams(){
    r = {0,0,double(imgSource.cols),double(imgSource.rows)};
    blurKernelSize = 5;
    cannyThreshold = 100;
    accumulatorThreshold = 50;
}

void HoughCircleDetector::readHoughParams(){

    FileStorage fs(paramFilePath.c_str(), FileStorage::READ);

    fs["cropRegion"] >> r;
    fs["blurKernelSize"] >> blurKernelSize;
    fs["cannyThreshold"] >> cannyThreshold;
    fs["accumulatorThreshold"] >> accumulatorThreshold;
    fs.release();
}

void HoughCircleDetector::cropImage(){
    imgSource = imgSource(r);
}

void HoughCircleDetector::convertToGray(){
    cvtColor(imgSource, imgGray, CV_BGR2GRAY);
}

void HoughCircleDetector::gaussianBlur() {
    GaussianBlur(imgSource, imgGray, Size(blurKernelSize, blurKernelSize), 2, 2);
}

bool HoughCircleDetector::detectCircles(cv::Mat &img){
    //std::vector<Vec3f> circles;
    cropImage();
    convertToGray();
    gaussianBlur();

    HoughCircles(imgGray, circles, HOUGH_GRADIENT, 1, imgGray.rows/8, cannyThreshold, accumulatorThreshold, 0, 0);

    imgDisplay = imgSource.clone();
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // Center drawing
        circle( imgDisplay, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // Circle drawing
        circle(imgDisplay, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }

    if (circles.size() < minNumCircles){
        return false;
    }
}
