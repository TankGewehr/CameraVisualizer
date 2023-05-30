#pragma once

#include <opencv2/opencv.hpp>

/**
 * 将欧拉角转换成旋转矩阵
 * @param roll
 * @param pitch
 * @param yaw
 */
cv::Mat eular_to_matrix(double roll, double pitch, double yaw);

/**
 * 使用相机在目标坐标系下的外参与相机内参，将3D空间坐标系中的点转换到2D图像坐标系
 * @param origin_xyz
 * @param extrinsic
 * @param intrinsic
 */
cv::Point2d ProjectPoint(cv::Point3d origin_xyz, cv::Mat extrinsic, cv::Mat intrinsic);

/**
 * 根据目标框的x、y、z、width、length、height、roll、pitch、yaw信息，获取目标框的角点
 * @param x
 * @param y
 * @param z
 * @param width
 * @param lenght
 * @param height
 * @param roll
 * @param pitch
 * @param yaw
 */
cv::Mat getBox(double x, double y, double z, double width, double length, double height, double roll, double pitch, double yaw);

/**
 * 使用相机在目标坐标系下的外参与相机内参，将目标坐标系下的目标框角点按框的顺序连线绘制在图像中
 * @param data
 * @param corners
 * @param extrinsic
 * @param intrinsic
 * @param colors
 */
void drawBoxCorners(cv::Mat data, cv::Mat corners, cv::Mat extrinsic, cv::Mat intrinsic, cv::Scalar colors);