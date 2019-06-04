//
// Created by eirik on 09.05.19.
//

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

void writeHoughParams(cv::Size imgDim, Rect2d r, int cannyThreshold, int accumulatorThreshold, const char* filePath)
{
    FileStorage fs(filePath, FileStorage::WRITE);
    fs << "imgDim" << imgDim;
    fs << "cropRegion" << r;
    fs << "blurKernelSize" << blurKernelSize;
    fs << "cannyThreshold" << cannyThreshold;
    fs << "accumulatorThreshold" << accumulatorThreshold;

    fs.release();
}

void printInfo(const char* argv){
    cout << "For a simple test, syntax is: " << argv << "  test" <<
    endl << "----------------------------" << endl;

    cout << "For saving of params for a specific image, syntax is: " << endl
    << argv << "  /image_name.{jpg|png}  " << "/hough_param_output.yml" <<
    endl << "----------------------------" << endl;
}

int main(int argc, char** argv)
{
    printInfo(argv[0]);

    Mat src, src_gray, imCrop;

    // Read the image
    string imageName, paramPath;
    cout << argv[1] << endl;

    if (argv[1] == string("test")){
        imageName = "/home/eirik/catkin_ws/src/hough_circle_detector/data/test_img.png";
        paramPath = "/home/eirik/catkin_ws/src/hough_circle_detector/data/test_params.yml";
    }
    else{

        imageName = argv[1];
        paramPath = argv[2];

        if (argc < 2){
            cout << "Too few arguments." << endl;
            return -1;
        }
    }

    cout << imageName << endl << paramPath << endl;
    src = imread(imageName, IMREAD_COLOR);

    if( src.empty() )
    {
        std::cerr<<"Invalid input image. Check image path."<< endl;
        return -1;
    }

    // Crop image
    cv::Size imgDim(src.cols, src.rows);
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
    cout << "Press q to quit and save configuration" << endl << endl;
    char key = 0;
    while(key != 'q' && key != 'Q')
    {
        cannyThreshold = std::max(cannyThreshold, 1);
        accumulatorThreshold = std::max(accumulatorThreshold, 1);

        //runs the detection, and update the display
        HoughDetection(src_gray, src, cannyThreshold, accumulatorThreshold);

        // get user key
        key = (char)waitKey(10);
    }

    cout << "imgDim" << imgDim << endl << endl;
    cout << "cropRegion: " << r << endl << endl;
    cout << "blurKernelSize: " << blurKernelSize << endl << endl;
    cout << "cannyThreshold: " <<  cannyThreshold << endl << endl;
    cout << "accumulatorThreshold: " <<  accumulatorThreshold << endl << endl;

    writeHoughParams(imgDim, r, cannyThreshold, accumulatorThreshold, paramPath.c_str());

    return 0;
}