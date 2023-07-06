#pragma once

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include "ros_interface/ObstacleList.h"
#include "sensors/Lidar.h"
#include "projection.h"

class LidarVisualizer
{
private:
    Lidar lidar_top;

    std::string obstacle_list_topic;
    std::string publish_topic;

    sensor_msgs::PointCloud2 msg;
    visualization_msgs::MarkerArray marker_array;

    ros::NodeHandle ros_nodehandle;
    ros::Publisher publisher;
    ros::Publisher publisher_rviz;

    std::vector<cv::Scalar> color_list;

public:
    LidarVisualizer(std::string lidar_top,
                    std::string frame_id,
                    std::string obstacle_list_topic,
                    std::string publish_topic);
    ~LidarVisualizer();

    void run();

    void callback(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                  const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                  Lidar &lidar);

    void publish();
};