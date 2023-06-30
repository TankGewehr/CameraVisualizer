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

    this->publisher = this->ros_nodehandle.advertise<sensor_msgs::PointCloud2>(this->publish_topic + this->lidar_top.getChannel(), 15);
    this->publisher_rviz = this->ros_nodehandle.advertise<visualization_msgs::MarkerArray>(this->publish_topic + "/MarkerArray", 0);
}

LidarVisualizer::~LidarVisualizer() = default;

void LidarVisualizer::run()
{
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_top(this->ros_nodehandle, this->lidar_top.getChannel(), 1);
    message_filters::Subscriber<ros_interface::ObstacleList> obstacle_list(this->ros_nodehandle, this->obstacle_list_topic, 1);

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ros_interface::ObstacleList>> lidar_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ros_interface::ObstacleList>(30), lidar_top, obstacle_list);

    // lidar_sync.setInterMessageLowerBound(ros::Duration(1));

    lidar_sync.registerCallback(boost::bind(&LidarVisualizer::callback, this, _1, _2, this->lidar_top));

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        this->publish();
        loop_rate.sleep();
    }
}

void LidarVisualizer::callback(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                               const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                               Lidar &lidar)
{
    // std_msgs::Header header = obstacle_list_msg->header;
    this->msg = *(lidar_msg);
    this->marker_array.markers.clear();
    int id = 0;

    std::vector<cv::Mat> normal_vecs, ds;
    std::vector<cv::Scalar> colors;

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

        cv::Mat corners = getBox(x, y, z, width, length, height, roll, pitch, yaw).t();

        int sub_type = obstacle.sub_type;

        visualization_msgs::Marker marker;
        marker.header.frame_id = this->lidar_top.getFrameId();
        marker.header.stamp = ros::Time();
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.color.r = this->color_list[sub_type][2] / 255;
        marker.color.g = this->color_list[sub_type][1] / 255;
        marker.color.b = this->color_list[sub_type][0] / 255;
        marker.lifetime = ros::Duration(0.2);

        geometry_msgs::Point point;
        point.x = corners.at<double>(0, 0);
        point.y = corners.at<double>(0, 1);
        point.z = corners.at<double>(0, 2);
        marker.points.push_back(point);
        point.x = corners.at<double>(1, 0);
        point.y = corners.at<double>(1, 1);
        point.z = corners.at<double>(1, 2);
        marker.points.push_back(point);

        point.x = corners.at<double>(1, 0);
        point.y = corners.at<double>(1, 1);
        point.z = corners.at<double>(1, 2);
        marker.points.push_back(point);
        point.x = corners.at<double>(2, 0);
        point.y = corners.at<double>(2, 1);
        point.z = corners.at<double>(2, 2);
        marker.points.push_back(point);

        point.x = corners.at<double>(2, 0);
        point.y = corners.at<double>(2, 1);
        point.z = corners.at<double>(2, 2);
        marker.points.push_back(point);
        point.x = corners.at<double>(3, 0);
        point.y = corners.at<double>(3, 1);
        point.z = corners.at<double>(3, 2);
        marker.points.push_back(point);

        point.x = corners.at<double>(3, 0);
        point.y = corners.at<double>(3, 1);
        point.z = corners.at<double>(3, 2);
        marker.points.push_back(point);
        point.x = corners.at<double>(0, 0);
        point.y = corners.at<double>(0, 1);
        point.z = corners.at<double>(0, 2);
        marker.points.push_back(point);

        point.x = corners.at<double>(4, 0);
        point.y = corners.at<double>(4, 1);
        point.z = corners.at<double>(4, 2);
        marker.points.push_back(point);
        point.x = corners.at<double>(5, 0);
        point.y = corners.at<double>(5, 1);
        point.z = corners.at<double>(5, 2);
        marker.points.push_back(point);

        point.x = corners.at<double>(5, 0);
        point.y = corners.at<double>(5, 1);
        point.z = corners.at<double>(5, 2);
        marker.points.push_back(point);
        point.x = corners.at<double>(6, 0);
        point.y = corners.at<double>(6, 1);
        point.z = corners.at<double>(6, 2);
        marker.points.push_back(point);

        point.x = corners.at<double>(6, 0);
        point.y = corners.at<double>(6, 1);
        point.z = corners.at<double>(6, 2);
        marker.points.push_back(point);
        point.x = corners.at<double>(7, 0);
        point.y = corners.at<double>(7, 1);
        point.z = corners.at<double>(7, 2);
        marker.points.push_back(point);

        point.x = corners.at<double>(7, 0);
        point.y = corners.at<double>(7, 1);
        point.z = corners.at<double>(7, 2);
        marker.points.push_back(point);
        point.x = corners.at<double>(4, 0);
        point.y = corners.at<double>(4, 1);
        point.z = corners.at<double>(4, 2);
        marker.points.push_back(point);

        point.x = corners.at<double>(0, 0);
        point.y = corners.at<double>(0, 1);
        point.z = corners.at<double>(0, 2);
        marker.points.push_back(point);
        point.x = corners.at<double>(4, 0);
        point.y = corners.at<double>(4, 1);
        point.z = corners.at<double>(4, 2);
        marker.points.push_back(point);

        point.x = corners.at<double>(1, 0);
        point.y = corners.at<double>(1, 1);
        point.z = corners.at<double>(1, 2);
        marker.points.push_back(point);
        point.x = corners.at<double>(5, 0);
        point.y = corners.at<double>(5, 1);
        point.z = corners.at<double>(5, 2);
        marker.points.push_back(point);

        point.x = corners.at<double>(2, 0);
        point.y = corners.at<double>(2, 1);
        point.z = corners.at<double>(2, 2);
        marker.points.push_back(point);
        point.x = corners.at<double>(6, 0);
        point.y = corners.at<double>(6, 1);
        point.z = corners.at<double>(6, 2);
        marker.points.push_back(point);

        point.x = corners.at<double>(3, 0);
        point.y = corners.at<double>(3, 1);
        point.z = corners.at<double>(3, 2);
        marker.points.push_back(point);
        point.x = corners.at<double>(7, 0);
        point.y = corners.at<double>(7, 1);
        point.z = corners.at<double>(7, 2);
        marker.points.push_back(point);

        point.x = (corners.at<double>(2, 0) + corners.at<double>(3, 0) + corners.at<double>(6, 0) + corners.at<double>(7, 0)) / 4;
        point.y = (corners.at<double>(2, 1) + corners.at<double>(3, 1) + corners.at<double>(6, 1) + corners.at<double>(7, 1)) / 4;
        point.z = (corners.at<double>(2, 2) + corners.at<double>(3, 2) + corners.at<double>(6, 2) + corners.at<double>(7, 2)) / 4;
        marker.points.push_back(point);
        point.x = (corners.at<double>(2, 0) + corners.at<double>(3, 0)) / 2;
        point.y = (corners.at<double>(2, 1) + corners.at<double>(3, 1)) / 2;
        point.z = (corners.at<double>(2, 2) + corners.at<double>(3, 2)) / 2;
        marker.points.push_back(point);

        point.x = x;
        point.y = y;
        point.z = z;
        marker.points.push_back(point);
        point.x = x + obstacle.velocity_vehicle.x;
        point.y = y + obstacle.velocity_vehicle.y;
        point.z = z + obstacle.velocity_vehicle.z;
        marker.points.push_back(point);

        this->marker_array.markers.push_back(marker);
    }

    std::cout << "[" << lidar_msg->header.stamp << "]: "
              << "(" << obstacle_list_msg->header.stamp - lidar_msg->header.stamp << ") "
              << lidar.getChannel() << std::endl;
}

void LidarVisualizer::publish()
{
    this->msg.header.stamp = ros::Time::now();
    this->msg.header.frame_id = this->lidar_top.getFrameId();
    this->publisher.publish(msg);
    std::cout << "[" << msg.header.stamp << "]: "
              << "(publish) "
              << this->publish_topic + this->lidar_top.getChannel() << std::endl;
    this->publisher_rviz.publish(this->marker_array);
    std::cout << "[" << msg.header.stamp << "]: "
              << "(publish) "
              << this->publish_topic + "/MarkerArray" << std::endl;
}

Lidar::Lidar(std::string channel, std::string frame_id)
{
    this->channel = channel;
    this->frame_id = frame_id;
}

Lidar::~Lidar() = default;

std::string Lidar::getChannel() const
{
    return this->channel;
}

std::string Lidar::getFrameId() const
{
    return this->frame_id;
}