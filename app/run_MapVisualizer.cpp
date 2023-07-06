#include "MapVisualizer.h"
#include <jsoncpp/json/json.h>

bool loadConfig(const std::string &filename,
                std::string &lidar_top,
                std::string &frame_id,
                std::string &lane_list_topic,
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
        if (root["MapVisualizer"].isNull() || root["MapVisualizer"].type() != Json::objectValue)
        {
            std::cout << "Error MapVisualizer type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["MapVisualizer"]["calibration_params_path"].isNull() || root["MapVisualizer"]["calibration_params_path"].type() != Json::objectValue || root["MapVisualizer"]["calibration_params_path"]["lidar_top"].isNull() || root["MapVisualizer"]["calibration_params_path"]["lidar_top"].type() != Json::stringValue)
        {
            std::cout << "Error calibration_params_path type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["MapVisualizer"]["frame_id"].isNull() || root["MapVisualizer"]["frame_id"].type() != Json::stringValue)
        {
            std::cout << "Error frame_id type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["MapVisualizer"]["lane_list_topic"].isNull() || root["MapVisualizer"]["lane_list_topic"].type() != Json::stringValue)
        {
            std::cout << "Error lane_list_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["MapVisualizer"]["publish_topic"].isNull() || root["MapVisualizer"]["publish_topic"].type() != Json::stringValue)
        {
            std::cout << "Error publish_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        lidar_top = root["MapVisualizer"]["calibration_params_path"]["lidar_top"].asString();
        frame_id = root["MapVisualizer"]["frame_id"].asString();
        lane_list_topic = root["MapVisualizer"]["lane_list_topic"].asString();
        publish_topic = root["MapVisualizer"]["publish_topic"].asString();
    }

    is.close();
    return true;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./MapVisualizer <config_json_path> \n"
                     "example:\n"
                     "\t./bin/MapVisualizer ./config/config.json"
                  << std::endl;
        return 1;
    }
    std::string lidar_top, frame_id, lane_list_topic, publish_topic;
    if (!loadConfig(argv[1], lidar_top, frame_id, lane_list_topic, publish_topic))
        return 1;

    ros::init(argc, argv, "MapVisualizer");
    MapVisualizer lidar_visualizer(
        lidar_top,
        frame_id,
        lane_list_topic,
        publish_topic);
    lidar_visualizer.run();
    return 0;
}