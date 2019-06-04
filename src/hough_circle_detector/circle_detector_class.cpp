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

HoughCircleDetector::HoughCircleDetector(std::string &detectorParamPath): paramFilePath(detectorParamPath){
    readHoughParams();
    minNumCircles = 1;
}

HoughCircleDetector::~HoughCircleDetector() = default;

cv::Mat HoughCircleDetector::getDisplayImage(){
    return imgDisplay;
}

bool HoughCircleDetector::getDetectionResult(cv::Mat &img){
    imgSource = img;
    detectCircles();
    if (circles.size() < minNumCircles){
        return false;
    }
}

void HoughCircleDetector::setDefaultHoughParams(){
    cout << "HoughCircleDetector: Passing default values, could not parse from file" << endl;
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

    cout << "BEFORE CROP: Image Width: " << imgSource.cols << "Height: " << imgSource.rows << endl;

    if(r.x >= 0 && r.y >= 0 && r.width + r.x < imgSource.cols && r.height + r.y < imgSource.rows)
    {
        imgCrop = imgSource(r);
    }
    else {
        cout << "Crop region invalid. Check if the image size allows for a crop region as specified."
             << "Leaving the image in original size. " << endl << endl;
        imgCrop = imgSource;
    }

    cout << "AFTER CROP: Image Width: " << imgCrop.cols << "Height: " << imgCrop.rows << endl;
}

void HoughCircleDetector::convertToGray(){
    cvtColor(imgCrop, imgGray, CV_BGR2GRAY);
}

void HoughCircleDetector::gaussianBlur() {
    GaussianBlur(imgGray, imgGray, Size(blurKernelSize, blurKernelSize), 2, 2);
}

bool HoughCircleDetector::detectCircles(){

    cropImage();
    convertToGray();
    gaussianBlur();

    HoughCircles(imgGray, circles, HOUGH_GRADIENT, 1, imgGray.rows/8, cannyThreshold, accumulatorThreshold, 0, 0);

    imgDisplay = imgCrop.clone();
    for(size_t i = 0; i < circles.size(); i++ )
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
