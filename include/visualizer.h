#pragma once

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include "ros_interface/ObstacleList.h"
#include "ros_interface/LaneList.h"
#include "sensors/Camera.h"
#include "sensors/Lidar.h"
#include "projection.h"

class visualizer
{
private:
    Camera cam_front_left;
    Camera cam_front;
    Camera cam_front_right;
    Camera cam_back_left;
    Camera cam_back;
    Camera cam_back_right;
    Lidar lidar_top;

    bool obstacle_enable;
    std::string obstacle_list_topic;
    bool lane_enable;
    std::string lane_list_topic;
    std::string publish_topic;

    cv::Mat image;
    sensor_msgs::CompressedImage compressed_image_msg;
    sensor_msgs::PointCloud2 point_cloud_msg;
    visualization_msgs::MarkerArray marker_array_msg;

    ros::NodeHandle ros_nodehandle;
    ros::Publisher compressed_image_publisher;
    ros::Publisher point_cloud_publisher;
    ros::Publisher marker_array_publisher;

    std::vector<cv::Scalar> obstacle_color_list;
    std::vector<std::string> obstacle_type_list;
    std::vector<cv::Scalar> lane_color_list;
    std::vector<std::string> lane_type_list;

public:
    visualizer(std::string cam_front_left,
               std::string cam_front,
               std::string cam_front_right,
               std::string cam_back_left,
               std::string cam_back,
               std::string cam_back_right,
               std::string lidar_top,
               bool obstacle_enable,
               std::string obstacle_list_topic,
               bool lane_enable,
               std::string lane_list_topic,
               std::string frame_id,
               std::string publish_topic);
    ~visualizer();

    void run(int rate = 10);

    void callback_camera(const sensor_msgs::CompressedImage::ConstPtr &compressed_image_msg,
                         const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                         const ros_interface::LaneList::ConstPtr &lane_list_msg,
                         Camera &camera,
                         int pos);

    void callback_camera_obstacle(const sensor_msgs::CompressedImage::ConstPtr &compressed_image_msg,
                                  const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                                  Camera &camera);

    void callback_camera_lane(const sensor_msgs::CompressedImage::ConstPtr &compressed_image_msg,
                              const ros_interface::LaneList::ConstPtr &lane_list_msg,
                              Camera &camera,
                              int pos);

    void callback_lidar(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                        const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                        const ros_interface::LaneList::ConstPtr &lane_list_msg,
                        Lidar &lidar);

    void callback_lidar_obstacle(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                                 const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                                 Lidar &lidar);

    void callback_lidar_lane(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                             const ros_interface::LaneList::ConstPtr &lane_list_msg,
                             Lidar &lidar);

    void publish();
};