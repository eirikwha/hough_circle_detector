//
// Created by eirik on 01.06.19.
//

#ifndef POSE_ESTIMATOR_CIRCLE_DETECTOR_NODE_H
#define POSE_ESTIMATOR_CIRCLE_DETECTOR_NODE_H

#include <ros/ros.h> //ALWAYS need to include this
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <std_srvs/SetBool.h>
#include <hough_circle_detector/circle_detector_class.h>


class CircleDetectorNode {
public:
    CircleDetectorNode(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
    ~CircleDetectorNode();

private:

    ros::NodeHandle nh;

    std::string packagePath = findPackagePath("hough_circle_detector");
    std::string detectorParamPath;

    std::string frameID;
    std::string circleDetectorServerName;
    std::string listenTopic;
    std::string imageStreamTopic;

    // Define messages
    cv::Mat imgDisplay;
    bool hasCorrectPose = false;

    ros::Subscriber subImage;
    ros::ServiceServer circleDetectorServer;
    ros::Publisher pubImage;

    // member methods as well:
    void initializeNodeParams();
    void initializeSubscribers();
    void initializePublishers();
    void initializeServices();
    void initializeDetectorParams();
    std::string findPackagePath(const std::string &packageName);

    void imageCallback(const sensor_msgs::Image &image);
    bool triggerCircleDetection(std_srvs::SetBoolRequest& request,
                       std_srvs::SetBoolResponse& response);
};


#endif //POSE_ESTIMATOR_CIRCLE_DETECTOR_NODE_H
