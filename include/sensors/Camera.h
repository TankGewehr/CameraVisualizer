#pragma once

#include <opencv2/opencv.hpp>
#include "CalibrationParam.h"

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

    cv::Mat undistorted_map_x; // x方向的映射map
    cv::Mat undistorted_map_y; // y方向的映射map

    cv::Mat image;             // 图像
    cv::Mat undistorted_image; // 去畸变后的图像

public:
    Camera(std::string calibration_param_path);
    ~Camera();

    void Update(cv::Mat image);

    void Draw(cv::Mat corners, cv::Scalar color, std::string type);

    void Draw(cv::Point2d point, cv::Scalar color);

    std::string getChannel() const;

    cv::Size getSize() const;

    cv::Mat getData() const;

    void Update_(cv::Mat image);

    cv::Mat getData_() const;

    void Draw_(cv::Point2d point, cv::Scalar color);
};