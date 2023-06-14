#include "LidarVisualizer.h"

LidarVisualizer::LidarVisualizer(std::string channel,
                                 std::string frame_id,
                                 std::string obstacle_list_topic,
                                 std::string publish_topic) : lidar_top(channel, frame_id)
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

    this->obstacle_list_topic = obstacle_list_topic;
    this->publish_topic = publish_topic;

    this->publisher = this->ros_nodehandle.advertise<sensor_msgs::PointCloud2>(this->publish_topic, 15);
}

LidarVisualizer::~LidarVisualizer() = default;

void LidarVisualizer::run()
{
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_top(this->ros_nodehandle, this->lidar_top.getChannel(), 30);
    message_filters::Subscriber<ros_interface::ObstacleList> obstacle_list(this->ros_nodehandle, this->obstacle_list_topic, 30);

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ros_interface::ObstacleList>> lidar_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ros_interface::ObstacleList>(30), lidar_top, obstacle_list);

    // lidar_sync.setInterMessageLowerBound(ros::Duration(1));

    lidar_sync.registerCallback(boost::bind(&LidarVisualizer::callback, this, _1, _2, this->lidar_top));

    ros::Rate loop_rate(15);
    while (ros::ok())
    {
        this->publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void LidarVisualizer::callback(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                               const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                               Lidar &lidar)
{
    // std_msgs::Header header = obstacle_list_msg->header;
    lidar.Update(lidar_msg);

    std::vector<cv::Mat> normal_vecs, ds;
    std::vector<cv::Scalar> colors;

    for (ros_interface::Obstacle obstacle : obstacle_list_msg->obstacle)
    {
        double x = obstacle.center_pos_vehicle.x;
        double y = obstacle.center_pos_vehicle.y;
        double z = obstacle.center_pos_vehicle.z;
        double width = obstacle.width;
        double lenght = obstacle.length;
        double height = obstacle.height;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = obstacle.theta_vehicle;

        cv::Mat corners = getBox(x, y, z, width, lenght, height, roll, pitch, yaw).t();

        int sub_type = obstacle.sub_type;

        cv::Mat normal_vec = cv::Mat(cv::Size(1, 6), CV_64FC3);
        normal_vec.at<cv::Vec3d>(0, 0) = (corners.at<cv::Vec3d>(0) - corners.at<cv::Vec3d>(1)).cross(corners.at<cv::Vec3d>(1) - corners.at<cv::Vec3d>(2));
        normal_vec.at<cv::Vec3d>(1, 0) = (corners.at<cv::Vec3d>(7) - corners.at<cv::Vec3d>(6)).cross(corners.at<cv::Vec3d>(6) - corners.at<cv::Vec3d>(5));
        normal_vec.at<cv::Vec3d>(2, 0) = (corners.at<cv::Vec3d>(0) - corners.at<cv::Vec3d>(3)).cross(corners.at<cv::Vec3d>(3) - corners.at<cv::Vec3d>(7));
        normal_vec.at<cv::Vec3d>(3, 0) = (corners.at<cv::Vec3d>(1) - corners.at<cv::Vec3d>(5)).cross(corners.at<cv::Vec3d>(5) - corners.at<cv::Vec3d>(6));
        normal_vec.at<cv::Vec3d>(4, 0) = (corners.at<cv::Vec3d>(0) - corners.at<cv::Vec3d>(4)).cross(corners.at<cv::Vec3d>(4) - corners.at<cv::Vec3d>(5));
        normal_vec.at<cv::Vec3d>(5, 0) = (corners.at<cv::Vec3d>(3) - corners.at<cv::Vec3d>(2)).cross(corners.at<cv::Vec3d>(2) - corners.at<cv::Vec3d>(6));

        cv::Mat d = cv::Mat(cv::Size(1, 6), CV_64FC1);
        d.at<double>(0, 0) = normal_vec.at<cv::Vec3d>(0, 0).dot(corners.at<cv::Vec3d>(0));
        d.at<double>(1, 0) = normal_vec.at<cv::Vec3d>(1, 0).dot(corners.at<cv::Vec3d>(7));
        d.at<double>(2, 0) = normal_vec.at<cv::Vec3d>(2, 0).dot(corners.at<cv::Vec3d>(0));
        d.at<double>(3, 0) = normal_vec.at<cv::Vec3d>(3, 0).dot(corners.at<cv::Vec3d>(1));
        d.at<double>(4, 0) = normal_vec.at<cv::Vec3d>(4, 0).dot(corners.at<cv::Vec3d>(0));
        d.at<double>(5, 0) = normal_vec.at<cv::Vec3d>(5, 0).dot(corners.at<cv::Vec3d>(3));

        normal_vecs.push_back(normal_vec);
        ds.push_back(d);
        colors.push_back(this->color_list[sub_type]);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_rgb = lidar.getData();
    for (size_t i = 0; i < point_cloud_rgb->points.size(); i++)
    {
        pcl::PointXYZRGB &point_rgb = point_cloud_rgb->points[i];
        point_rgb.r = point_rgb.g = point_rgb.b = 200;

        for (size_t j = 0; j < normal_vecs.size(); j++)
        {
            bool in_box = true;
            for (int k = 0; k < 6; k++)
            {
                if ((point_rgb.x * normal_vecs[j].at<cv::Vec3d>(k, 0)[0] +
                     point_rgb.y * normal_vecs[j].at<cv::Vec3d>(k, 0)[1] +
                     point_rgb.z * normal_vecs[j].at<cv::Vec3d>(k, 0)[2] -
                     ds[j].at<double>(k, 0)) >= 0)
                {
                    in_box = false;
                    break;
                }
            }
            if (in_box)
            {
                point_rgb.r = colors[j][2];
                point_rgb.g = colors[j][1];
                point_rgb.b = colors[j][0];
            }
        }
    }

    std::cout << "[" << lidar_msg->header.stamp << "]: "
              << "(" << obstacle_list_msg->header.stamp - lidar_msg->header.stamp << ") "
              << lidar.getChannel() << std::endl;
}

void LidarVisualizer::publish()
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*(this->lidar_top.getData()), msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = this->lidar_top.getFrameId();
    this->publisher.publish(msg);
}

Lidar::Lidar(std::string channel, std::string frame_id) : point_cloud(new pcl::PointCloud<pcl::PointXYZI>), point_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    this->channel = channel;
    this->frame_id = frame_id;
}

Lidar::~Lidar() = default;

void Lidar::Update(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg)
{
    pcl::fromROSMsg(*lidar_msg, *(this->point_cloud));
    pcl::copyPointCloud(*(this->point_cloud), *(this->point_cloud_rgb));
}

std::string Lidar::getChannel() const
{
    return this->channel;
}

std::string Lidar::getFrameId() const
{
    return this->frame_id;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Lidar::getData() const
{
    return this->point_cloud_rgb;
}