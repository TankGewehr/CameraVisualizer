#include "projection.h"

cv::Mat eular_to_matrix(double roll, double pitch, double yaw)
{
    return (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0) *
           (cv::Mat_<double>(3, 3) << cos(pitch), 0.0, sin(pitch), 0.0, 1.0, 0.0, -sin(pitch), 0.0, cos(pitch)) *
           (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, cos(roll), -sin(roll), 0.0, sin(roll), cos(roll));
}

cv::Point2d ProjectPoint(cv::Point3d origin_xyz, cv::Mat extrinsic, cv::Mat intrinsic)
{
    cv::Mat origin_point = (cv::Mat_<double>(4, 1) << origin_xyz.x, origin_xyz.y, origin_xyz.z, 1.0);
    cv::Mat camera_point = (extrinsic * origin_point)(cv::Range(0, 3), cv::Range(0, 1));

    if (camera_point.at<double>(2, 0) < 0.1)
    {
        return cv::Point2d(-1.0, -1.0);
    }

    cv::Mat result_point = intrinsic * camera_point;
    double result_x, result_y;
    result_x = result_point.at<double>(0, 0) / result_point.at<double>(2, 0);
    result_y = result_point.at<double>(1, 0) / result_point.at<double>(2, 0);
    return cv::Point2d(result_x, result_y);
}

// 根据目标框的x、y、z、width、length、height、roll、pitch、yaw信息，获取目标框的角点
cv::Mat getBox(double x, double y, double z, double width, double length, double height, double roll, double pitch, double yaw)
{
    cv::Mat corners(3, 8, CV_64FC1);

    // 根据目标框的长宽高生成目标框的8个顶点
    double *corners_x = corners.ptr<double>(0);
    corners_x[0] = corners_x[1] = corners_x[2] = corners_x[3] = length / 2;
    corners_x[4] = corners_x[5] = corners_x[6] = corners_x[7] = -length / 2;

    double *corners_y = corners.ptr<double>(1);
    corners_y[0] = corners_y[3] = corners_y[4] = corners_y[7] = width / 2;
    corners_y[1] = corners_y[2] = corners_y[5] = corners_y[6] = -width / 2;

    double *corners_z = corners.ptr<double>(2);
    corners_z[0] = corners_z[1] = corners_z[4] = corners_z[5] = height / 2;
    corners_z[2] = corners_z[3] = corners_z[6] = corners_z[7] = -height / 2;

    // 根据目标框的旋转角旋转顶点
    corners = eular_to_matrix(roll, pitch, yaw) * corners;

    // 根据目标框的位置平移顶点
    for (int col = 0; col < corners.cols; col++)
    {
        corners.at<double>(0, col) += x;
        corners.at<double>(1, col) += y;
        corners.at<double>(2, col) += z;
    }
    return corners;
}

void drawBoxCorners(cv::Mat data, cv::Mat corners, cv::Mat extrinsic, cv::Mat intrinsic, cv::Scalar colors)
{
    // 将目标框的顶点转换到图像坐标系
    std::vector<cv::Point2d> corners_xy;
    for (int col = 0; col < corners.cols; col++)
    {
        corners_xy.push_back(
            ProjectPoint(
                cv::Point3d(
                    corners.at<double>(0, col),
                    corners.at<double>(1, col),
                    corners.at<double>(2, col)),
                extrinsic,
                intrinsic));
    }

    // 在图像中绘制目标框的边
    int err = 0;
    for (int i = 0; i < 8; i++)
    {
        if (corners_xy[i].x >= data.cols || corners_xy[i].x <= 0 || corners_xy[i].y >= data.rows || corners_xy[i].y <= 0)
            err++;
    }
    if (err >= 7)
        return;
    for (int i = 0; i < 4; i++)
    {
        cv::line(data, cv::Point(corners_xy[i].x, corners_xy[i].y), cv::Point(corners_xy[i + 4].x, corners_xy[i + 4].y), colors, 1, 8);
        cv::line(data, cv::Point(corners_xy[2 * i].x, corners_xy[2 * i].y), cv::Point(corners_xy[2 * i + 1].x, corners_xy[2 * i + 1].y), colors, 1, 8);
        if (i < 2)
        {
            cv::line(data, cv::Point(corners_xy[i].x, corners_xy[i].y), cv::Point(corners_xy[3 - i].x, corners_xy[3 - i].y), colors, 1, 8);
        }
        else
        {
            cv::line(data, cv::Point(corners_xy[i + 2].x, corners_xy[i + 2].y), cv::Point(corners_xy[9 - i].x, corners_xy[9 - i].y), colors, 1, 8);
        }
    }

    return;
}