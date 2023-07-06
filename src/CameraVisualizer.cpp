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
    this->obstacle_list_topic = obstacle_list_topic;
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
    this->msg.format = "jpeg";

    this->publisher = this->ros_nodehandle.advertise<sensor_msgs::CompressedImage>(this->publish_topic + "/compressed", 15);

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
}

CameraVisualizer::~CameraVisualizer() = default;

void CameraVisualizer::run()
{
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front_left(this->ros_nodehandle, this->cam_front_left.getChannel(), 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front(this->ros_nodehandle, this->cam_front.getChannel(), 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front_right(this->ros_nodehandle, this->cam_front_right.getChannel(), 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back_left(this->ros_nodehandle, this->cam_back_left.getChannel(), 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back(this->ros_nodehandle, this->cam_back.getChannel(), 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back_right(this->ros_nodehandle, this->cam_back_right.getChannel(), 1);
    message_filters::Subscriber<ros_interface::ObstacleList> obstacle_list(this->ros_nodehandle, this->obstacle_list_topic, 1);

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
        ros::spinOnce();
        this->publish();
        loop_rate.sleep();
    }
}

void CameraVisualizer::callback(const sensor_msgs::CompressedImage::ConstPtr &compressed_image_msg,
                                const ros_interface::ObstacleList::ConstPtr &obstacle_list_msg,
                                Camera &camera)
{
    camera.Update(cv::imdecode(cv::Mat(compressed_image_msg->data), cv::IMREAD_COLOR));

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

    cv::imencode(".jpg", this->image, this->msg.data);
    this->msg.header.stamp = ros::Time::now();
    this->publisher.publish(this->msg);
    std::cout << "[" << this->msg.header.stamp << "]: "
              << "(publish) "
              << this->publish_topic << std::endl;
}