#pragma once

#include <iostream>
#include <fstream>
#include <algorithm>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "ros_interface/LaneList.h"
#include "sensors/Camera.h"

class RoadVisualizer
{
private:
    Camera cam_front;
    Camera cam_back;

    std::string lane_list_topic;
    std::string publish_topic;

    cv::Mat image;
    sensor_msgs::CompressedImage msg;

    ros::NodeHandle ros_nodehandle;
    ros::Publisher publisher;

    std::vector<cv::Scalar> color_list;
    std::vector<std::string> type_list;

public:
    RoadVisualizer(std::string cam_front,
                   std::string cam_back,
                   std::string lane_list_topic,
                   std::string publish_topic);
    ~RoadVisualizer();

    void run();

    void callback(const sensor_msgs::CompressedImage::ConstPtr &compressed_image_msg,
                  const ros_interface::LaneList::ConstPtr &lane_list_msg,
                  Camera &camera,
                  int pos);

    void publish();
};