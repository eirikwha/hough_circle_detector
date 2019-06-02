//
// Created by eirik on 01.06.19.
//

#include <circle_detector_node.h>

// Use pose est package as template
// Class based
// Rqt image view, maybe added in launch file
// Ros image stream
// Sub, pub and service call

using namespace std;

CircleDetectorNode::CircleDetectorNode(ros::NodeHandle* nodehandle):nh(*nodehandle){

    ROS_INFO("in class constructor of CircleDetectorNode");

    initializeDetectorParams();
    initializeServices();
    initializePublishers();

}

CircleDetectorNode::~CircleDetectorNode() = default;

void CircleDetectorNode::initializeDetectorParams(){

    ROS_INFO("Initializing the parameters");

    if (!nh.getParam("detector_params", detectorParamPath)) // defined private in .h
    {
        detectorParamPath = packagePath + "/data/houghparams.yml";
        ROS_ERROR("CircleDetectorNode Class: Could not parse detector params. Setting default from /data.");
    }
    if (!nh.getParam("circle_detector_server_name", circleDetectorServerName))
    {
        circleDetectorServerName = "circle_detector_server";
        ROS_ERROR("PoseEstimator Class: Could not parse pose estimator server name. Setting default.");
    }

    if (!nh.getParam("image_in_topic", listenTopic))
    {
        listenTopic = "/image";
        ROS_ERROR("PoseEstimator Class: Could not parse pointcloud input topic. Setting default.");
    }

    if (!nh.getParam("image_out_topic", imageStreamTopic))
    {
        imageStreamTopic = "/image_out";
        ROS_ERROR("PoseEstimator Class: Could not parse pose array output topic. Setting default.");
    }
}

void CircleDetectorNode::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");

    /// Create a ROS subscriber for the output PoseArray (for marker visualization)
    subImage = nh.subscribe(listenTopic, 1, &CircleDetectorNode::imageCallback, this);
}

void CircleDetectorNode::initializeServices()
{
    ROS_INFO("Initializing Services");

    circleDetectorServer = nh.advertiseService(circleDetectorServerName,
                                               &CircleDetectorNode::triggerCircleDetection, this);
}

void CircleDetectorNode::initializePublishers()
{
    ROS_INFO("Initializing Publishers");

    /// Create a ROS publisher for the output PoseArray
    pubImage = nh.advertise<sensor_msgs::Image>(imageStreamTopic, 1);
}

std::string CircleDetectorNode::findPackagePath(const string &packageName){
    return ros::package::getPath(packageName);
}


void CircleDetectorNode::imageCallback(const sensor_msgs::CompressedImage &image){
    ROS_INFO("Image callback activated. Detecting circles");

    HoughCircleDetector houghDetector(detectorParamPath); // todo - should not be put here!!
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        imgDisplay = (cv_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    bool detectionResult = houghDetector.getDetectionResult(imgDisplay);
    imgDisplay = houghDetector.getDisplayImage();

    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time
    cv_bridge::CvImage cv_ptr2 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, imgDisplay);

    pubImage.publish(cv_ptr2.toImageMsg()); // TODO: convert to ros image

    if (detectionResult){
        ROS_INFO("TEST: Has correct pose");
        hasCorrectPose = true;
    }
    else{
        ROS_INFO("TEST: Has wrong pose");
    }

    subImage.shutdown();
}

bool CircleDetectorNode::triggerCircleDetection(std_srvs::TriggerRequest &request,
                                                std_srvs::TriggerResponse &response) {
    ROS_INFO("Service callback activated");
    subImage = nh.subscribe(listenTopic, 1, &CircleDetectorNode::imageCallback, this);
    response.success = 1;
    response.message = "Circle detector initiated.";
    return true;
}


int main(int argc, char** argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "circle_detector"); //node name
    ros::NodeHandle nh("~"); // private nodehandle to set private params

    ROS_INFO("main: instantiating an object of type PoseEstimator");
    CircleDetectorNode circleDetector(&nh);  //instantiate an PoseEstimator object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
}

