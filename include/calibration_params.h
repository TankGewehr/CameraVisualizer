#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <jsoncpp/json/json.h>

/**
 * 读取内参矩阵、畸变参数、图像大小，默认读取的是去畸变后的图像再次标定的内参"undistort_intrinsic"以及畸变参数"undistort_distortion"，如果需要读取内参"intrinsic"以及畸变系数"distortion"，请将undistorted参数设置为false
 * @param filename
 * @param intrinsic
 * @param distortion
 * @param undistorted
 */
bool loadIntrinsic(const std::string &filename, cv::Mat &intrinsic, cv::Mat &distortion, cv::Size &image_size, bool undistorted = true);

/**
 * 读取外参矩阵
 * @param filename
 * @param extrinsic
 */
bool loadExtrinsic(const std::string &filename, cv::Mat &extrinsic);

/**
 * 修改并保存外参，仅修改外参部分（平移和旋转）
 * @param filename
 * @param extrinsic
 */
bool saveExtrinsic(const std::string &filename, cv::Mat extrinsic);