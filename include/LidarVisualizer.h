#pragma once

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ros_interface/ObstacleList.h"
#include "projection.h"

class Lidar
{
private:
    std::string channel;
    std::string frame_id;

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_rgb;

public:
    Lidar(std::string channel, std::string frame_id);
    ~Lidar();

    void Update(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg);

    std::string getChannel() const;

    std::string getFrameId() const;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getData() const;
};

class LidarVisualizer
{
private:
    Lidar lidar_top;
    std::string obstacle_list_topic;
    std::string publish_topic;

    ros::NodeHandle ros_nodehandle;
    ros::Subscriber subscriber;
    ros::Publisher publisher;

    std::vector<cv::Scalar> color_list;

    visualization_msgs::MarkerArray marker_array;
    ros::Publisher publisher_rviz;

public:
    LidarVisualizer(std::string channel,
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