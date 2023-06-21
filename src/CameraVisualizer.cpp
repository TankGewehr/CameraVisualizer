#include "CameraVisualizer.h"

CameraVisualizer::CameraVisualizer(std::string cam_front_left,
                                   std::string cam_front,
                                   std::string cam_front_right,
                                   std::string cam_back_left,
                                   std::string cam_back,
                                   std::string cam_back_right,
                                   std::string obstacle_list_topic,
                                   std::string publish_topic) : cam_front_left(cam_front_left),
                                                                cam_front(cam_front),
                                                                cam_front_right(cam_front_right),
                                                                cam_back_left(cam_back_left),
                                                                cam_back(cam_back),
                                                                cam_back_right(cam_back_right)
{
    this->color_list.clear();
    this->color_list.push_back(cv::Scalar(190, 190, 190));
    this->color_list.push_back(cv::Scalar(190, 190, 190));
    this->color_list.push_back(cv::Scalar(190, 190, 190));
    this->color_list.push_back(cv::Scalar(0, 255, 0));
    this->color_list.push_back(cv::Scalar(255, 255, 0));
    this->color_list.push_back(cv::Scalar(34, 34, 178));
    this->color_list.push_back(cv::Scalar(240, 34, 160));
    this->color_list.push_back(cv::Scalar(0, 165, 255));
    this->color_list.push_back(cv::Scalar(0, 127, 255));
    this->color_list.push_back(cv::Scalar(0, 102, 205));
    this->color_list.push_back(cv::Scalar(255, 0, 0));
    this->color_list.push_back(cv::Scalar(79, 79, 47));
    this->color_list.push_back(cv::Scalar(203, 192, 255));
    this->color_list.push_back(cv::Scalar(255, 0, 255));

    this->type_list.clear();
    this->type_list.push_back("Unknown");
    this->type_list.push_back("Unknown");
    this->type_list.push_back("Unknown");
    this->type_list.push_back("Car");
    this->type_list.push_back("Van");
    this->type_list.push_back("Truck");
    this->type_list.push_back("Bus");
    this->type_list.push_back("Cyclist");
    this->type_list.push_back("Motocyclist");
    this->type_list.push_back("Tricyclist");
    this->type_list.push_back("Pedestrian");
    this->type_list.push_back("Trafficcone");
    this->type_list.push_back("Pillar");
    this->type_list.push_back("Speed_bump");

    int rows = this->cam_front.getSize().height +
               this->cam_back.getSize().height +
               std::max(this->cam_front_left.getSize().height, this->cam_front_right.getSize().height) / 2 +
               std::max(this->cam_back_left.getSize().height, this->cam_back_right.getSize().height) / 2;
    int cols = std::max({
        this->cam_front.getSize().width,
        this->cam_back.getSize().width,
        this->cam_front_left.getSize().width / 2 + this->cam_front_right.getSize().width / 2,
        this->cam_back_left.getSize().width / 2 + this->cam_back_right.getSize().width / 2,
    });

    this->image_size = cv::Size(cols, rows);
    this->image = cv::Mat::zeros(this->image_size, CV_8UC3);

    this->obstacle_list_topic = obstacle_list_topic;
    this->publish_topic = publish_topic;

    this->publisher = this->ros_nodehandle.advertise<sensor_msgs::CompressedImage>(this->publish_topic, 15);
}

CameraVisualizer::~CameraVisualizer() = default;

void CameraVisualizer::run()
{
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front_left(this->ros_nodehandle, this->cam_front_left.getChannel(), 30);
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front(this->ros_nodehandle, this->cam_front.getChannel(), 30);
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front_right(this->ros_nodehandle, this->cam_front_right.getChannel(), 30);
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back_left(this->ros_nodehandle, this->cam_back_left.getChannel(), 30);
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back(this->ros_nodehandle, this->cam_back.getChannel(), 30);
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back_right(this->ros_nodehandle, this->cam_back_right.getChannel(), 30);
    message_filters::Subscriber<ros_interface::ObstacleList> obstacle_list(this->ros_nodehandle, this->obstacle_list_topic, 30);

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_front_left_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_front_left, obstacle_list);
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_front_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_front, obstacle_list);
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_front_right_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_front_right, obstacle_list);
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_back_left_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_back_left, obstacle_list);
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_back_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_back, obstacle_list);
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_back_right_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_back_right, obstacle_list);

    // cam_front_left_sync.setInterMessageLowerBound(ros::Duration(1));
    // cam_front_sync.setInterMessageLowerBound(ros::Duration(1));
    // cam_front_right_sync.setInterMessageLowerBound(ros::Duration(1));
    // cam_back_left_sync.setInterMessageLowerBound(ros::Duration(1));
    // cam_back_sync.setInterMessageLowerBound(ros::Duration(1));
    // cam_back_right_sync.setInterMessageLowerBound(ros::Duration(1));

    cam_front_left_sync.registerCallback(boost::bind(&CameraVisualizer::callback, this, _1, _2, this->cam_front_left));
    cam_front_sync.registerCallback(boost::bind(&CameraVisualizer::callback, this, _1, _2, this->cam_front));
    cam_front_right_sync.registerCallback(boost::bind(&CameraVisualizer::callback, this, _1, _2, this->cam_front_right));
    cam_back_left_sync.registerCallback(boost::bind(&CameraVisualizer::callback, this, _1, _2, this->cam_back_left));
    cam_back_sync.registerCallback(boost::bind(&CameraVisualizer::callback, this, _1, _2, this->cam_back));
    cam_back_right_sync.registerCallback(boost::bind(&CameraVisualizer::callback, this, _1, _2, this->cam_back_right));

    ros::Rate loop_rate(15);
    while (ros::ok())
    {
        this->publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void CameraVisualizer::callback(const sensor_msgs::CompressedImage::ConstPtr &compressed_image_msg,
                                const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                                Camera &camera)
{
    // std_msgs::Header header = obstacle_list_msg->header;
    camera.Update(cv_bridge::toCvCopy(*compressed_image_msg, sensor_msgs::image_encodings::BGR8)->image);
    for (ros_interface::Obstacle obstacle : obstacle_list_msg->obstacle)
    {
        double x = obstacle.center_pos_vehicle.x;
        double y = obstacle.center_pos_vehicle.y;
        double z = obstacle.center_pos_vehicle.z;
        double width = obstacle.width;
        double length = obstacle.length;
        double height = obstacle.height;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = obstacle.theta_vehicle;

        cv::Mat corners = getBox(x, y, z, width, length, height, roll, pitch, yaw);

        int sub_type = obstacle.sub_type;

        camera.Draw(corners, this->color_list[sub_type], this->type_list[sub_type]);
    }

    std::cout << "[" << compressed_image_msg->header.stamp << "]: "
              << "(" << obstacle_list_msg->header.stamp - compressed_image_msg->header.stamp << ") "
              << camera.getChannel() << std::endl;
}

void CameraVisualizer::publish()
{
    cv::Mat front_left, front_right, back_left, back_right;
    cv::resize(this->cam_front_left.getData(), front_left, this->cam_front_left.getSize() / 2);
    cv::resize(this->cam_front_right.getData(), front_right, this->cam_front_right.getSize() / 2);
    cv::resize(this->cam_back_left.getData(), back_left, this->cam_back_left.getSize() / 2);
    cv::resize(this->cam_back_right.getData(), back_right, this->cam_back_right.getSize() / 2);
    this->cam_front.getData().copyTo(
        this->image(
            cv::Rect(
                0,
                0,
                this->cam_front.getSize().width,
                this->cam_front.getSize().height)));
    this->cam_back.getData().copyTo(
        this->image(
            cv::Rect(
                0,
                this->cam_front.getSize().height,
                this->cam_back.getSize().width,
                this->cam_back.getSize().height)));
    front_left.copyTo(
        this->image(
            cv::Rect(
                0,
                this->cam_front.getSize().height + this->cam_back.getSize().height,
                this->cam_front_left.getSize().width / 2,
                this->cam_front_left.getSize().height / 2)));
    front_right.copyTo(
        this->image(
            cv::Rect(
                this->cam_front_left.getSize().width / 2,
                this->cam_front.getSize().height + this->cam_back.getSize().height,
                this->cam_front_right.getSize().width / 2,
                this->cam_front_right.getSize().height / 2)));
    back_left.copyTo(
        this->image(
            cv::Rect(
                0,
                this->cam_front.getSize().height + this->cam_back.getSize().height + std::max(this->cam_front_left.getSize().height, this->cam_front_right.getSize().height) / 2,
                this->cam_back_left.getSize().width / 2,
                this->cam_back_left.getSize().height / 2)));

    back_right.copyTo(
        this->image(
            cv::Rect(
                this->cam_back_left.getSize().width / 2,
                this->cam_front.getSize().height + this->cam_back.getSize().height + std::max(this->cam_front_left.getSize().height, this->cam_front_right.getSize().height) / 2,
                this->cam_back.getSize().width / 2,
                this->cam_back.getSize().height / 2)));

    sensor_msgs::CompressedImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->image).toCompressedImageMsg();
    this->publisher.publish(msg);
}

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

    cv::initUndistortRectifyMap(this->intrinsic, this->distortion, cv::Mat::eye(cv::Size(3, 3), CV_64F), this->intrinsic, this->image_size, CV_16SC2, this->map_x, this->map_y);

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