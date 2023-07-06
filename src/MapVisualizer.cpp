#include "MapVisualizer.h"

MapVisualizer::MapVisualizer(std::string lidar_top,
                             std::string frame_id,
                             std::string lane_list_topic,
                             std::string publish_topic) : lidar_top(lidar_top)
{
    this->lidar_top.setFrameId(frame_id);
    this->lane_list_topic = lane_list_topic;
    this->publish_topic = publish_topic;

    this->publisher = this->ros_nodehandle.advertise<sensor_msgs::PointCloud2>(this->publish_topic + this->lidar_top.getChannel(), 15);
    this->publisher_rviz = this->ros_nodehandle.advertise<visualization_msgs::MarkerArray>(this->publish_topic + "/MarkerArray", 0);

    this->color_list.clear();
    this->color_list.push_back(cv::Scalar(255, 0, 0));
    this->color_list.push_back(cv::Scalar(0, 255, 0));
    this->color_list.push_back(cv::Scalar(0, 0, 255));
    this->color_list.push_back(cv::Scalar(255, 255, 0));
    this->color_list.push_back(cv::Scalar(255, 0, 255));
    this->color_list.push_back(cv::Scalar(0, 255, 255));
    this->color_list.push_back(cv::Scalar(255, 255, 255));
    this->color_list.push_back(cv::Scalar(100, 255, 0));
    this->color_list.push_back(cv::Scalar(100, 0, 255));
    this->color_list.push_back(cv::Scalar(255, 100, 0));
    this->color_list.push_back(cv::Scalar(0, 100, 255));
    this->color_list.push_back(cv::Scalar(255, 0, 100));
    this->color_list.push_back(cv::Scalar(0, 255, 100));
}

MapVisualizer::~MapVisualizer() = default;

void MapVisualizer::run()
{
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_top(this->ros_nodehandle, this->lidar_top.getChannel(), 1);
    message_filters::Subscriber<ros_interface::LaneList> lane_list(this->ros_nodehandle, this->lane_list_topic, 1);

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ros_interface::LaneList>> lidar_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ros_interface::LaneList>(100), lidar_top, lane_list);

    // lidar_sync.setInterMessageLowerBound(ros::Duration(1));

    lidar_sync.registerCallback(boost::bind(&MapVisualizer::callback, this, _1, _2, this->lidar_top));

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        this->publish();
        loop_rate.sleep();
    }
}

void MapVisualizer::callback(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                             const ros_interface::LaneList::ConstPtr &lane_list_msg,
                             Lidar &lidar)
{
    this->msg = *(lidar_msg);
    this->marker_array.markers.clear();
    int id = 0;

    std::vector<cv::Mat> normal_vecs, ds;
    std::vector<cv::Scalar> colors;

    for (ros_interface::LaneLine laneline : lane_list_msg->camera_laneline)
    {
        int lane_type = laneline.lane_type;

        visualization_msgs::Marker marker;
        marker.header.frame_id = lidar.getFrameId();
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
        marker.color.r = this->color_list[lane_type][2] / 255;
        marker.color.g = this->color_list[lane_type][1] / 255;
        marker.color.b = this->color_list[lane_type][0] / 255;
        marker.lifetime = ros::Duration(0.2);

        for (ros_interface::Point3D pt_vehicle : laneline.pts_vehicle)
        {
            cv::Mat origin_point = (cv::Mat_<double>(4, 1) << pt_vehicle.x, pt_vehicle.y, pt_vehicle.z, 1.0);
            cv::Mat lidar_point = (lidar.getExtrinsic() * origin_point)(cv::Range(0, 3), cv::Range(0, 1));
            geometry_msgs::Point point;
            point.x = lidar_point.at<double>(0, 0);
            point.y = lidar_point.at<double>(1, 0);
            point.z = lidar_point.at<double>(2, 0);
            marker.points.push_back(point);
            marker.points.push_back(point);
        }
        marker.points.erase(marker.points.begin());
        marker.points.pop_back();

        this->marker_array.markers.push_back(marker);
    }

    std::cout << "[" << lidar_msg->header.stamp << "]: "
              << "(" << lane_list_msg->header.stamp - lidar_msg->header.stamp << ") "
              << lidar.getChannel() << std::endl;
}

void MapVisualizer::publish()
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