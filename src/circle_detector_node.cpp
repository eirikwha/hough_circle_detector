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
    initializeSubscribers();
    initializePublishers();
    initializeServices();
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


void CircleDetectorNode::imageCallback(const sensor_msgs::Image &image){
    ROS_INFO("Image callback activated. Detecting circles");
    HoughCircleDetector houghDetector(detectorParamPath.c_str()); // todo - should not be put here!!
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        imgDisplay =  cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if (houghDetector.getDetectionResult(imgDisplay)){
        imgDisplay = houghDetector.getDisplayImage();
        hasCorrectPose = true;
    }
}

bool CircleDetectorNode::triggerCircleDetection(std_srvs::SetBoolRequest &request,
                                                std_srvs::SetBoolResponse &response) {
    ROS_INFO("Service callback activated");
    subImage = nh.subscribe(listenTopic, 1, &CircleDetectorNode::imageCallback, this);
    response.success = 1; // TODO: Need to define a bool message
    return true;
}


int main(int argc, char** argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "circle_detector"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type PoseEstimator");
    CircleDetectorNode circleDetector(&nh);  //instantiate an PoseEstimator object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
}

