#pragma once

#include <iostream>
#include <fstream>
#include <algorithm>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "ros_interface/ObstacleList.h"
#include "calibration_params.h"
#include "projection.h"

class Camera
{
private:
    std::string channel; // rostopic
    cv::Size image_size; // 图像大小

    cv::Mat intrinsic;  // 内参矩阵
    cv::Mat distortion; // 畸变参数

    cv::Mat undistort_intrinsic;  // 去畸变后重新标定的内参矩阵
    cv::Mat undistort_distortion; // 去畸变后重新标定的畸变系数

    cv::Mat extrinsic; // 外参矩阵

    cv::Mat map_x; // x方向的映射map
    cv::Mat map_y; // y方向的映射map

    cv::Mat image;             // 图像
    cv::Mat undistorted_image; // 去畸变后的图像
public:
    Camera(std::string intrinsic_and_extrinsic_json_path);
    ~Camera();

    void Update(cv::Mat image);

    void Draw(cv::Mat corners, cv::Scalar color, std::string type);

    std::string getChannel() const;

    cv::Size getSize() const;

    cv::Mat getData() const;
};

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

    ros::NodeHandle ros_nodehandle;
    ros::Subscriber subscriber;
    image_transport::ImageTransport it_;
    image_transport::Publisher publisher;

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

    void callback(const sensor_msgs::CompressedImage::ConstPtr &cam_front_left_msg,
                  const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                  Camera &camera);

    void publish();
};