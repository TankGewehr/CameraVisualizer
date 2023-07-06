#pragma once

#include <iostream>
#include <fstream>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include "ros_interface/ObstacleList.h"
#include "sensors/Camera.h"
#include "projection.h"

class CameraVisualizer
{
private:
    Camera cam_front_left;
    Camera cam_front;
    Camera cam_front_right;
    Camera cam_back_left;
    Camera cam_back;
    Camera cam_back_right;

    std::string obstacle_list_topic;
    std::string publish_topic;

    cv::Mat image;
    sensor_msgs::CompressedImage msg;

    ros::NodeHandle ros_nodehandle;
    ros::Publisher publisher;

    std::vector<cv::Scalar> color_list;
    std::vector<std::string> type_list;

public:
    CameraVisualizer(std::string cam_front_left,
                     std::string cam_front,
                     std::string cam_front_right,
                     std::string cam_back_left,
                     std::string cam_back,
                     std::string cam_back_right,
                     std::string obstacle_list_topic,
                     std::string publish_topic);
    ~CameraVisualizer();

    void run();

    void callback(const sensor_msgs::CompressedImage::ConstPtr &compressed_image_msg,
                  const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                  Camera &camera);

    void publish();
};