#pragma once

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include "ros_interface/LaneList.h"
#include "sensors/Lidar.h"
#include "projection.h"

class MapVisualizer
{
private:
    Lidar lidar_top;

    std::string lane_list_topic;
    std::string publish_topic;

    sensor_msgs::PointCloud2 msg;
    visualization_msgs::MarkerArray marker_array;

    ros::NodeHandle ros_nodehandle;
    ros::Publisher publisher;
    ros::Publisher publisher_rviz;

    std::vector<cv::Scalar> color_list;

public:
    MapVisualizer(std::string lidar_top,
                  std::string frame_id,
                  std::string lane_list_topic,
                  std::string publish_topic);
    ~MapVisualizer();

    void run();

    void callback(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                  const ros_interface::LaneList::ConstPtr &lane_list_msg,
                  Lidar &lidar);

    void publish();
};