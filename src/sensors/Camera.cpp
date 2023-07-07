#include "sensors/Camera.h"

Camera::Camera(std::string intrinsic_and_extrinsic_json_path)
{
    Json::Reader reader;
    Json::Value root;

    std::ifstream is(intrinsic_and_extrinsic_json_path, std::ios::binary);
    if (!is.is_open())
    {
        std::cout << "Error opening file:" << intrinsic_and_extrinsic_json_path << std::endl;
    }
    else
    {
        if (reader.parse(is, root))
        {
            if (!root["channel"].isNull() && root["channel"].type() == Json::stringValue)
            {
                this->channel = root["channel"].asString();
            }
            else
            {
                std::cout << "Error channel type:" << intrinsic_and_extrinsic_json_path << std::endl;
            }
        }

        is.close();
    }

    loadIntrinsic(intrinsic_and_extrinsic_json_path, this->intrinsic, this->distortion, this->image_size, false);
    loadIntrinsic(intrinsic_and_extrinsic_json_path, this->undistort_intrinsic, this->undistort_distortion, this->image_size);

    cv::Mat extrinsic;
    loadExtrinsic(intrinsic_and_extrinsic_json_path, extrinsic);
    this->extrinsic = extrinsic.inv();

    cv::initUndistortRectifyMap(this->intrinsic, this->distortion, cv::Mat::eye(cv::Size(3, 3), CV_64F), this->intrinsic, this->image_size, CV_32FC1, this->map_x, this->map_y);

    this->image = cv::Mat::zeros(this->image_size, CV_8UC3);
    this->undistorted_image = cv::Mat::zeros(this->image_size, CV_8UC3);
}

Camera::~Camera() = default;

void Camera::Update(cv::Mat image)
{
    assert(image.size() == this->image_size);
    image.copyTo(this->image);
    cv::remap(this->image, this->undistorted_image, this->map_x, this->map_y, cv::INTER_LINEAR);
}

void Camera::Draw(cv::Mat corners, cv::Scalar color, std::string type)
{
    // 将目标框的顶点转换到图像坐标系
    std::vector<cv::Point2d> corners_xy;
    for (int col = 0; col < corners.cols; col++)
    {
        cv::Mat origin_point = (cv::Mat_<double>(4, 1) << corners.at<double>(0, col), corners.at<double>(1, col), corners.at<double>(2, col), 1.0);
        cv::Mat camera_point = (this->extrinsic * origin_point)(cv::Range(0, 3), cv::Range(0, 1));

        if (camera_point.at<double>(2, 0) < 0.1)
        {
            return;
        }

        cv::Mat result_point = this->undistort_intrinsic * camera_point;
        double result_x, result_y;
        result_x = result_point.at<double>(0, 0) / result_point.at<double>(2, 0);
        result_y = result_point.at<double>(1, 0) / result_point.at<double>(2, 0);

        corners_xy.push_back(cv::Point2d(result_x, result_y));
    }

    // 在图像中绘制目标框的边、头指向、类别
    cv::line(this->undistorted_image, corners_xy[0], corners_xy[1], color, 2, 8);
    cv::line(this->undistorted_image, corners_xy[1], corners_xy[2], color, 2, 8);
    cv::line(this->undistorted_image, corners_xy[2], corners_xy[3], color, 2, 8);
    cv::line(this->undistorted_image, corners_xy[3], corners_xy[0], color, 2, 8);
    cv::line(this->undistorted_image, corners_xy[4], corners_xy[5], color, 2, 8);
    cv::line(this->undistorted_image, corners_xy[5], corners_xy[6], color, 2, 8);
    cv::line(this->undistorted_image, corners_xy[6], corners_xy[7], color, 2, 8);
    cv::line(this->undistorted_image, corners_xy[7], corners_xy[4], color, 2, 8);
    cv::line(this->undistorted_image, corners_xy[0], corners_xy[4], color, 2, 8);
    cv::line(this->undistorted_image, corners_xy[1], corners_xy[5], color, 2, 8);
    cv::line(this->undistorted_image, corners_xy[2], corners_xy[6], color, 2, 8);
    cv::line(this->undistorted_image, corners_xy[3], corners_xy[7], color, 2, 8);

    cv::arrowedLine(this->undistorted_image, (corners_xy[2] + corners_xy[3] + corners_xy[6] + corners_xy[7]) / 4, (corners_xy[2] + corners_xy[3]) / 2, color, 2, 8);
    cv::putText(this->undistorted_image, type, cv::Point(corners_xy[7].x, corners_xy[7].y), 5, 0.6, color, 0.3);

    return;
}

std::string Camera::getChannel() const
{
    return this->channel;
}

cv::Size Camera::getSize() const
{
    return this->image_size;
}

cv::Mat Camera::getData() const
{
    return this->undistorted_image;
}

void Camera::Draw(cv::Point2d point, cv::Scalar color)
{
    cv::circle(this->undistorted_image,
               cv::Point2f(
                   this->map_x.at<float>(point.y, point.x),
                   this->map_y.at<float>(point.y, point.x)),
               2,
               color,
               2);

    return;
}

void Camera::Update_(cv::Mat image)
{
    assert(image.size() == this->image_size);
    image.copyTo(this->image);
}

cv::Mat Camera::getData_() const
{
    return this->image;
}

void Camera::Draw_(cv::Point2d point, cv::Scalar color)
{
    cv::circle(this->image,point,2,color,2);

    return;
}