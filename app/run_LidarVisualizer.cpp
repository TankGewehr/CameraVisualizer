#include "LidarVisualizer.h"
#include <jsoncpp/json/json.h>

bool loadConfig(const std::string &filename,
                std::string &channel,
                std::string &frame_id,
                std::string &obstacle_list_topic,
                std::string &publish_topic)
{
    Json::Reader reader;
    Json::Value root;

    std::ifstream is(filename, std::ios::binary);
    if (!is.is_open())
    {
        std::cout << "Error opening file:" << filename << std::endl;
        return false;
    }

    if (reader.parse(is, root))
    {
        if (root["LidarVisualizer"].isNull() || root["LidarVisualizer"].type() != Json::objectValue)
        {
            std::cout << "Error LidarVisualizer type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["LidarVisualizer"]["channel"].isNull() || root["LidarVisualizer"]["channel"].type() != Json::stringValue)
        {
            std::cout << "Error channel type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["LidarVisualizer"]["frame_id"].isNull() || root["LidarVisualizer"]["frame_id"].type() != Json::stringValue)
        {
            std::cout << "Error frame_id type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["LidarVisualizer"]["obstacle_list_topic"].isNull() || root["LidarVisualizer"]["obstacle_list_topic"].type() != Json::stringValue)
        {
            std::cout << "Error obstacle_list_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["LidarVisualizer"]["publish_topic"].isNull() || root["LidarVisualizer"]["publish_topic"].type() != Json::stringValue)
        {
            std::cout << "Error publish_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        channel = root["LidarVisualizer"]["channel"].asString();
        frame_id = root["LidarVisualizer"]["frame_id"].asString();
        obstacle_list_topic = root["LidarVisualizer"]["obstacle_list_topic"].asString();
        publish_topic = root["LidarVisualizer"]["publish_topic"].asString();
    }

    is.close();
    return true;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./LidarVisualizerApp <config_json_path> \n"
                     "example:\n"
                     "\t./bin/LidarVisualizerApp ./config/config.json"
                  << std::endl;
        return 1;
    }
    std::string channel, frame_id, obstacle_list_topic, publish_topic;
    if (!loadConfig(argv[1], channel, frame_id, obstacle_list_topic, publish_topic))
        return 1;

    ros::init(argc, argv, "LidarVisualizer");
    LidarVisualizer *lidar_visualizer = new LidarVisualizer(
        channel,
        frame_id,
        obstacle_list_topic,
        publish_topic);
    lidar_visualizer->run();
    return 0;
}