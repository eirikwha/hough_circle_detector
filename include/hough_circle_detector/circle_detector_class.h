//
// Created by eirik on 02.06.19.
//
#pragma once
#ifndef HOUGH_CIRCLE_DETECTOR_CIRCLE_DETECTOR_CLASS_H
#define HOUGH_CIRCLE_DETECTOR_CIRCLE_DETECTOR_CLASS_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

class HoughCircleDetector{
public:
    HoughCircleDetector();
    HoughCircleDetector(std::string &detectorParamPath);

    ~HoughCircleDetector();

    std::string windowName = "Hough Circle Detection";
    cv::Mat getDisplayImage();
    bool getDetectionResult(cv::Mat &img);

private:

    std::string paramFilePath;
    cv::Rect2d r;
    int blurKernelSize;
    int cannyThreshold;
    int accumulatorThreshold;

    std::vector<cv::Vec3f> circles;
    int minNumCircles;

    cv::Mat imgSource;
    cv::Mat imgGray;
    cv::Mat imgDisplay;

    void setDefaultHoughParams();
    void readHoughParams();
    void cropImage();
    void convertToGray();
    void gaussianBlur();
    bool detectCircles(cv::Mat &img);
};


#endif //HOUGH_CIRCLE_DETECTOR_CIRCLE_DETECTOR_CLASS_H
