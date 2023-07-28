#include "visualizer.h"

visualizer::visualizer(std::string cam_front_left,
                       std::string cam_front,
                       std::string cam_front_right,
                       std::string cam_back_left,
                       std::string cam_back,
                       std::string cam_back_right,
                       std::string lidar_top,
                       bool obstacle_enable,
                       std::string obstacle_list_topic,
                       bool lane_enable,
                       std::string lane_list_topic,
                       std::string frame_id,
                       std::string publish_topic) : cam_front_left(cam_front_left),
                                                    cam_front(cam_front),
                                                    cam_front_right(cam_front_right),
                                                    cam_back_left(cam_back_left),
                                                    cam_back(cam_back),
                                                    cam_back_right(cam_back_right),
                                                    lidar_top(lidar_top)
{

    this->obstacle_enable = obstacle_enable;
    this->obstacle_list_topic = obstacle_list_topic;
    this->lane_enable = lane_enable;
    this->lane_list_topic = lane_list_topic;
    this->lidar_top.setFrameId(frame_id);
    this->publish_topic = publish_topic;

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

    this->image = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC3);
    this->compressed_image_msg.format = "jpeg";

    this->compressed_image_publisher = this->ros_nodehandle.advertise<sensor_msgs::CompressedImage>(this->publish_topic + "/compressed", 0);
    this->point_cloud_publisher = this->ros_nodehandle.advertise<sensor_msgs::PointCloud2>(this->publish_topic + "/PointCloud2", 0);
    this->marker_array_publisher = this->ros_nodehandle.advertise<visualization_msgs::MarkerArray>(this->publish_topic + "/MarkerArray", 0);

    this->obstacle_color_list.clear();
    this->obstacle_color_list.push_back(cv::Scalar(190, 190, 190));
    this->obstacle_color_list.push_back(cv::Scalar(190, 190, 190));
    this->obstacle_color_list.push_back(cv::Scalar(190, 190, 190));
    this->obstacle_color_list.push_back(cv::Scalar(0, 255, 0));
    this->obstacle_color_list.push_back(cv::Scalar(255, 255, 0));
    this->obstacle_color_list.push_back(cv::Scalar(34, 34, 178));
    this->obstacle_color_list.push_back(cv::Scalar(240, 34, 160));
    this->obstacle_color_list.push_back(cv::Scalar(0, 165, 255));
    this->obstacle_color_list.push_back(cv::Scalar(0, 127, 255));
    this->obstacle_color_list.push_back(cv::Scalar(0, 102, 205));
    this->obstacle_color_list.push_back(cv::Scalar(255, 0, 0));
    this->obstacle_color_list.push_back(cv::Scalar(79, 79, 47));
    this->obstacle_color_list.push_back(cv::Scalar(203, 192, 255));
    this->obstacle_color_list.push_back(cv::Scalar(255, 0, 255));

    this->obstacle_type_list.clear();
    this->obstacle_type_list.push_back("Unknown");
    this->obstacle_type_list.push_back("Unknown");
    this->obstacle_type_list.push_back("Unknown");
    this->obstacle_type_list.push_back("Car");
    this->obstacle_type_list.push_back("Van");
    this->obstacle_type_list.push_back("Truck");
    this->obstacle_type_list.push_back("Bus");
    this->obstacle_type_list.push_back("Cyclist");
    this->obstacle_type_list.push_back("Motocyclist");
    this->obstacle_type_list.push_back("Tricyclist");
    this->obstacle_type_list.push_back("Pedestrian");
    this->obstacle_type_list.push_back("Trafficcone");
    this->obstacle_type_list.push_back("Pillar");
    this->obstacle_type_list.push_back("Speed_bump");

    this->lane_color_list.clear();
    this->lane_color_list.push_back(cv::Scalar(255, 0, 0));
    this->lane_color_list.push_back(cv::Scalar(0, 255, 0));
    this->lane_color_list.push_back(cv::Scalar(0, 0, 255));
    this->lane_color_list.push_back(cv::Scalar(255, 255, 0));
    this->lane_color_list.push_back(cv::Scalar(255, 0, 255));
    this->lane_color_list.push_back(cv::Scalar(0, 255, 255));
    this->lane_color_list.push_back(cv::Scalar(255, 255, 255));
    this->lane_color_list.push_back(cv::Scalar(100, 255, 0));
    this->lane_color_list.push_back(cv::Scalar(100, 0, 255));
    this->lane_color_list.push_back(cv::Scalar(255, 100, 0));
    this->lane_color_list.push_back(cv::Scalar(0, 100, 255));
    this->lane_color_list.push_back(cv::Scalar(255, 0, 100));
    this->lane_color_list.push_back(cv::Scalar(0, 255, 100));

    this->lane_type_list.clear();
}

visualizer::~visualizer() = default;

void visualizer::run(int rate)
{
    if (this->obstacle_enable && this->lane_enable)
    {
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front_left(this->ros_nodehandle, this->cam_front_left.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front(this->ros_nodehandle, this->cam_front.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front_right(this->ros_nodehandle, this->cam_front_right.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back_left(this->ros_nodehandle, this->cam_back_left.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back(this->ros_nodehandle, this->cam_back.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back_right(this->ros_nodehandle, this->cam_back_right.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_top(this->ros_nodehandle, this->lidar_top.getChannel(), 1);
        message_filters::Subscriber<ros_interface::ObstacleList> obstacle_list(this->ros_nodehandle, this->obstacle_list_topic, 1);
        message_filters::Subscriber<ros_interface::LaneList> lane_list(this->ros_nodehandle, this->lane_list_topic, 1);

        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_front_left_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_front_left, obstacle_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList, ros_interface::LaneList>> cam_front_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList, ros_interface::LaneList>(30), cam_front, obstacle_list, lane_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_front_right_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_front_right, obstacle_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_back_left_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_back_left, obstacle_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList, ros_interface::LaneList>> cam_back_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList, ros_interface::LaneList>(30), cam_back, obstacle_list, lane_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_back_right_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_back_right, obstacle_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ros_interface::ObstacleList, ros_interface::LaneList>> lidar_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ros_interface::ObstacleList, ros_interface::LaneList>(30), lidar_top, obstacle_list, lane_list);

        cam_front_left_sync.registerCallback(boost::bind(&visualizer::callback_camera_obstacle, this, _1, _2, this->cam_front_left));
        cam_front_sync.registerCallback(boost::bind(&visualizer::callback_camera, this, _1, _2, _3, this->cam_front, 1));
        cam_front_right_sync.registerCallback(boost::bind(&visualizer::callback_camera_obstacle, this, _1, _2, this->cam_front_right));
        cam_back_left_sync.registerCallback(boost::bind(&visualizer::callback_camera_obstacle, this, _1, _2, this->cam_back_left));
        cam_back_sync.registerCallback(boost::bind(&visualizer::callback_camera, this, _1, _2, _3, this->cam_back, -1));
        cam_back_right_sync.registerCallback(boost::bind(&visualizer::callback_camera_obstacle, this, _1, _2, this->cam_back_right));
        lidar_sync.registerCallback(boost::bind(&visualizer::callback_lidar, this, _1, _2, _3, this->lidar_top));

        ros::Rate loop_rate(rate);
        while (ros::ok())
        {
            ros::spinOnce();
            this->publish();
            loop_rate.sleep();
        }
    }
    else if (this->obstacle_enable)
    {
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front_left(this->ros_nodehandle, this->cam_front_left.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front(this->ros_nodehandle, this->cam_front.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front_right(this->ros_nodehandle, this->cam_front_right.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back_left(this->ros_nodehandle, this->cam_back_left.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back(this->ros_nodehandle, this->cam_back.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back_right(this->ros_nodehandle, this->cam_back_right.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_top(this->ros_nodehandle, this->lidar_top.getChannel(), 1);
        message_filters::Subscriber<ros_interface::ObstacleList> obstacle_list(this->ros_nodehandle, this->obstacle_list_topic, 1);

        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_front_left_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_front_left, obstacle_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_front_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_front, obstacle_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_front_right_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_front_right, obstacle_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_back_left_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_back_left, obstacle_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_back_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_back, obstacle_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>> cam_back_right_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::ObstacleList>(30), cam_back_right, obstacle_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ros_interface::ObstacleList>> lidar_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ros_interface::ObstacleList>(30), lidar_top, obstacle_list);

        cam_front_left_sync.registerCallback(boost::bind(&visualizer::callback_camera_obstacle, this, _1, _2, this->cam_front_left));
        cam_front_sync.registerCallback(boost::bind(&visualizer::callback_camera_obstacle, this, _1, _2, this->cam_front));
        cam_front_right_sync.registerCallback(boost::bind(&visualizer::callback_camera_obstacle, this, _1, _2, this->cam_front_right));
        cam_back_left_sync.registerCallback(boost::bind(&visualizer::callback_camera_obstacle, this, _1, _2, this->cam_back_left));
        cam_back_sync.registerCallback(boost::bind(&visualizer::callback_camera_obstacle, this, _1, _2, this->cam_back));
        cam_back_right_sync.registerCallback(boost::bind(&visualizer::callback_camera_obstacle, this, _1, _2, this->cam_back_right));
        lidar_sync.registerCallback(boost::bind(&visualizer::callback_lidar_obstacle, this, _1, _2, this->lidar_top));

        ros::Rate loop_rate(rate);
        while (ros::ok())
        {
            ros::spinOnce();
            this->publish();
            loop_rate.sleep();
        }
    }
    else if (this->lane_enable)
    {
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front(this->ros_nodehandle, this->cam_front.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back(this->ros_nodehandle, this->cam_back.getChannel(), 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_top(this->ros_nodehandle, this->lidar_top.getChannel(), 1);
        message_filters::Subscriber<ros_interface::LaneList> lane_list(this->ros_nodehandle, this->lane_list_topic, 1);

        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::LaneList>> cam_front_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::LaneList>(30), cam_front, lane_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::LaneList>> cam_back_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::LaneList>(30), cam_back, lane_list);
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ros_interface::LaneList>> lidar_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ros_interface::LaneList>(30), lidar_top, lane_list);

        cam_front_sync.registerCallback(boost::bind(&visualizer::callback_camera_lane, this, _1, _2, this->cam_front, 1));
        cam_back_sync.registerCallback(boost::bind(&visualizer::callback_camera_lane, this, _1, _2, this->cam_back, -1));
        lidar_sync.registerCallback(boost::bind(&visualizer::callback_lidar_lane, this, _1, _2, this->lidar_top));

        ros::Rate loop_rate(rate);
        while (ros::ok())
        {
            ros::spinOnce();
            this->publish();
            loop_rate.sleep();
        }
    }
    else
    {
        std::cout << "At least one of the values of \"obstacle_enable\" and \"lane_enable\" must be true" << std::endl;
    }
}

void visualizer::callback_camera(const sensor_msgs::CompressedImage::ConstPtr &compressed_image_msg,
                                 const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                                 const ros_interface::LaneList::ConstPtr &lane_list_msg,
                                 Camera &camera,
                                 int pos)
{
    camera.Update(cv::imdecode(cv::Mat(compressed_image_msg->data), cv::IMREAD_COLOR));

    for (ros_interface::Obstacle obstacle : obstacle_list_msg->obstacle)
    {
        cv::Mat corners = getBox(
            obstacle.center_pos_vehicle.x,
            obstacle.center_pos_vehicle.y,
            obstacle.center_pos_vehicle.z,
            obstacle.width, obstacle.length,
            obstacle.height,
            0.0,
            0.0,
            obstacle.theta_vehicle);

        camera.Draw(
            corners,
            this->obstacle_color_list[obstacle.sub_type],
            this->obstacle_type_list[obstacle.sub_type]);
    }

    for (ros_interface::LaneLine laneline : lane_list_msg->camera_laneline)
    {
        for (ros_interface::Point2D pts : laneline.pts_image)
        {
            if (pts.x * pos > 0 && pts.y * pos > 0)
            {
                camera.Draw(
                    cv::Point2d(pts.x * pos, pts.y * pos),
                    this->lane_color_list[laneline.lane_type]);
            }
        }
    }

    std::cout << "[" << compressed_image_msg->header.stamp << "]: ("
              << obstacle_list_msg->header.stamp - compressed_image_msg->header.stamp << " , "
              << lane_list_msg->header.stamp - compressed_image_msg->header.stamp << ") "
              << camera.getChannel() << std::endl;
}

void visualizer::callback_camera_obstacle(const sensor_msgs::CompressedImage::ConstPtr &compressed_image_msg,
                                          const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                                          Camera &camera)
{
    camera.Update(cv::imdecode(cv::Mat(compressed_image_msg->data), cv::IMREAD_COLOR));

    for (ros_interface::Obstacle obstacle : obstacle_list_msg->obstacle)
    {
        cv::Mat corners = getBox(
            obstacle.center_pos_vehicle.x,
            obstacle.center_pos_vehicle.y,
            obstacle.center_pos_vehicle.z,
            obstacle.width, obstacle.length,
            obstacle.height,
            0.0,
            0.0,
            obstacle.theta_vehicle);

        camera.Draw(
            corners,
            this->obstacle_color_list[obstacle.sub_type],
            this->obstacle_type_list[obstacle.sub_type]);
    }

    std::cout << "[" << compressed_image_msg->header.stamp << "]: "
              << "(" << obstacle_list_msg->header.stamp - compressed_image_msg->header.stamp << ") "
              << camera.getChannel() << std::endl;
}

void visualizer::callback_camera_lane(const sensor_msgs::CompressedImage::ConstPtr &compressed_image_msg,
                                      const ros_interface::LaneList::ConstPtr &lane_list_msg,
                                      Camera &camera,
                                      int pos)
{
    camera.Update(cv::imdecode(cv::Mat(compressed_image_msg->data), cv::IMREAD_COLOR));

    for (ros_interface::LaneLine laneline : lane_list_msg->camera_laneline)
    {
        for (ros_interface::Point2D pts : laneline.pts_image)
        {
            if (pts.x * pos > 0 && pts.y * pos > 0)
            {
                camera.Draw(
                    cv::Point2d(pts.x * pos, pts.y * pos),
                    this->lane_color_list[laneline.lane_type]);
            }
        }
    }

    std::cout << "[" << compressed_image_msg->header.stamp << "]: "
              << "(" << lane_list_msg->header.stamp - compressed_image_msg->header.stamp << ") "
              << camera.getChannel() << std::endl;
}

void visualizer::callback_lidar(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                                const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                                const ros_interface::LaneList::ConstPtr &lane_list_msg,
                                Lidar &lidar)
{
    this->point_cloud_msg = *(lidar_msg);
    this->marker_array_msg.markers.clear();
    int id = 0;

    for (ros_interface::Obstacle obstacle : obstacle_list_msg->obstacle)
    {
        cv::Mat corners = getBox(
                              obstacle.center_pos_vehicle.x,
                              obstacle.center_pos_vehicle.y,
                              obstacle.center_pos_vehicle.z,
                              obstacle.width, obstacle.length,
                              obstacle.height,
                              0.0,
                              0.0,
                              obstacle.theta_vehicle)
                              .t();

        visualization_msgs::Marker marker;
        marker.header.frame_id = lidar.getFrameId();
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.color.r = this->obstacle_color_list[obstacle.sub_type][2] / 255;
        marker.color.g = this->obstacle_color_list[obstacle.sub_type][1] / 255;
        marker.color.b = this->obstacle_color_list[obstacle.sub_type][0] / 255;
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

        point.x = obstacle.center_pos_vehicle.x;
        point.y = obstacle.center_pos_vehicle.y;
        point.z = obstacle.center_pos_vehicle.z;
        marker.points.push_back(point);
        point.x = obstacle.center_pos_vehicle.x + obstacle.velocity_vehicle.x;
        point.y = obstacle.center_pos_vehicle.y + obstacle.velocity_vehicle.y;
        point.z = obstacle.center_pos_vehicle.z + obstacle.velocity_vehicle.z;
        marker.points.push_back(point);

        this->marker_array_msg.markers.push_back(marker);
    }

    for (ros_interface::LaneLine laneline : lane_list_msg->camera_laneline)
    {
        int lane_type = laneline.lane_type;

        visualization_msgs::Marker marker;
        marker.header.frame_id = lidar.getFrameId();
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.color.r = this->lane_color_list[lane_type][2] / 255;
        marker.color.g = this->lane_color_list[lane_type][1] / 255;
        marker.color.b = this->lane_color_list[lane_type][0] / 255;
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

        this->marker_array_msg.markers.push_back(marker);
    }

    std::cout << "[" << lidar_msg->header.stamp << "]: ("
              << obstacle_list_msg->header.stamp - lidar_msg->header.stamp << " , "
              << lane_list_msg->header.stamp - lidar_msg->header.stamp << ") "
              << lidar.getChannel() << std::endl;
}

void visualizer::callback_lidar_obstacle(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                                         const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                                         Lidar &lidar)
{
    this->point_cloud_msg = *(lidar_msg);
    this->marker_array_msg.markers.clear();
    int id = 0;

    for (ros_interface::Obstacle obstacle : obstacle_list_msg->obstacle)
    {
        cv::Mat corners = getBox(
                              obstacle.center_pos_vehicle.x,
                              obstacle.center_pos_vehicle.y,
                              obstacle.center_pos_vehicle.z,
                              obstacle.width, obstacle.length,
                              obstacle.height,
                              0.0,
                              0.0,
                              obstacle.theta_vehicle)
                              .t();

        visualization_msgs::Marker marker;
        marker.header.frame_id = lidar.getFrameId();
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.color.r = this->obstacle_color_list[obstacle.sub_type][2] / 255;
        marker.color.g = this->obstacle_color_list[obstacle.sub_type][1] / 255;
        marker.color.b = this->obstacle_color_list[obstacle.sub_type][0] / 255;
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

        point.x = obstacle.center_pos_vehicle.x;
        point.y = obstacle.center_pos_vehicle.y;
        point.z = obstacle.center_pos_vehicle.z;
        marker.points.push_back(point);
        point.x = obstacle.center_pos_vehicle.x + obstacle.velocity_vehicle.x;
        point.y = obstacle.center_pos_vehicle.y + obstacle.velocity_vehicle.y;
        point.z = obstacle.center_pos_vehicle.z + obstacle.velocity_vehicle.z;
        marker.points.push_back(point);

        this->marker_array_msg.markers.push_back(marker);
    }

    std::cout << "[" << lidar_msg->header.stamp << "]: "
              << "(" << obstacle_list_msg->header.stamp - lidar_msg->header.stamp << ") "
              << lidar.getChannel() << std::endl;
}

void visualizer::callback_lidar_lane(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                                     const ros_interface::LaneList::ConstPtr &lane_list_msg,
                                     Lidar &lidar)
{
    this->point_cloud_msg = *(lidar_msg);
    this->marker_array_msg.markers.clear();
    int id = 0;

    for (ros_interface::LaneLine laneline : lane_list_msg->camera_laneline)
    {
        int lane_type = laneline.lane_type;

        visualization_msgs::Marker marker;
        marker.header.frame_id = lidar.getFrameId();
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.color.r = this->lane_color_list[lane_type][2] / 255;
        marker.color.g = this->lane_color_list[lane_type][1] / 255;
        marker.color.b = this->lane_color_list[lane_type][0] / 255;
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

        this->marker_array_msg.markers.push_back(marker);
    }

    std::cout << "[" << lidar_msg->header.stamp << "]: "
              << "(" << lane_list_msg->header.stamp - lidar_msg->header.stamp << ") "
              << lidar.getChannel() << std::endl;
}

void visualizer::publish()
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

    cv::imencode(".jpg", this->image, this->compressed_image_msg.data);
    this->compressed_image_msg.header.stamp = ros::Time::now();
    this->compressed_image_publisher.publish(compressed_image_msg);
    std::cout << "[" << this->compressed_image_msg.header.stamp << "]: "
              << "(publish) "
              << this->publish_topic + "/compressed" << std::endl;

    this->point_cloud_msg.header.frame_id = this->lidar_top.getFrameId();
    this->point_cloud_msg.header.stamp = ros::Time::now();
    this->point_cloud_publisher.publish(this->point_cloud_msg);
    std::cout << "[" << this->point_cloud_msg.header.stamp << "]: "
              << "(publish) "
              << this->publish_topic + "/PointCloud2" << std::endl;

    this->marker_array_publisher.publish(this->marker_array_msg);
    std::cout << "[" << ros::Time::now() << "]: "
              << "(publish) "
              << this->publish_topic + "/MarkerArray" << std::endl;
}