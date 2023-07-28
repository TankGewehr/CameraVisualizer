#include "RoadVisualizer.h"

RoadVisualizer::RoadVisualizer(std::string cam_front,
                               std::string cam_back,
                               std::string lane_list_topic,
                               std::string publish_topic) : cam_front(cam_front),
                                                            cam_back(cam_back)

{
    this->lane_list_topic = lane_list_topic;
    this->publish_topic = publish_topic;

    int rows = this->cam_front.getSize().height +
               this->cam_back.getSize().height;
    int cols = std::max({this->cam_front.getSize().width,
                         this->cam_back.getSize().width});

    this->image = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC3);
    this->msg.format = "jpeg";

    this->publisher = this->ros_nodehandle.advertise<sensor_msgs::CompressedImage>(this->publish_topic + "/compressed", 15);

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

RoadVisualizer::~RoadVisualizer() = default;

void RoadVisualizer::run()
{
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_front(this->ros_nodehandle, this->cam_front.getChannel(), 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam_back(this->ros_nodehandle, this->cam_back.getChannel(), 1);
    message_filters::Subscriber<ros_interface::LaneList> lane_list(this->ros_nodehandle, this->lane_list_topic, 1);

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::LaneList>> cam_front_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::LaneList>(100), cam_front, lane_list);
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::LaneList>> cam_back_sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, ros_interface::LaneList>(100), cam_back, lane_list);

    // cam_front_sync.setInterMessageLowerBound(ros::Duration(1));
    // cam_back_sync.setInterMessageLowerBound(ros::Duration(1));

    cam_front_sync.registerCallback(boost::bind(&RoadVisualizer::callback, this, _1, _2, this->cam_front, 1));
    cam_back_sync.registerCallback(boost::bind(&RoadVisualizer::callback, this, _1, _2, this->cam_back, -1));

    ros::Rate loop_rate(15);
    while (ros::ok())
    {
        ros::spinOnce();
        this->publish();
        loop_rate.sleep();
    }
}

void RoadVisualizer::callback(const sensor_msgs::CompressedImage::ConstPtr &compressed_image_msg,
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
                camera.Draw_(cv::Point2d(pts.x * pos, pts.y * pos), this->color_list[laneline.lane_type]);
            }
        }
    }

    std::cout << "[" << compressed_image_msg->header.stamp << "]: "
              << "(" << lane_list_msg->header.stamp - compressed_image_msg->header.stamp << ") "
              << camera.getChannel() << std::endl;
}

void RoadVisualizer::publish()
{
    this->cam_front.getData_().copyTo(
        this->image(
            cv::Rect(
                0,
                0,
                this->cam_front.getSize().width,
                this->cam_front.getSize().height)));
    this->cam_back.getData_().copyTo(
        this->image(
            cv::Rect(
                0,
                this->cam_front.getSize().height,
                this->cam_back.getSize().width,
                this->cam_back.getSize().height)));

    cv::imencode(".jpg", this->image, this->msg.data);
    this->msg.header.stamp = ros::Time::now();
    this->publisher.publish(this->msg);
    std::cout << "[" << this->msg.header.stamp << "]: "
              << "(publish) "
              << this->publish_topic << std::endl;
}