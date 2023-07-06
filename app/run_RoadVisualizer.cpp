#include "RoadVisualizer.h"

bool loadConfig(const std::string &filename,
                std::string &cam_front,
                std::string &cam_back,
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
        if (root["RoadVisualizer"].isNull() || root["RoadVisualizer"].type() != Json::objectValue)
        {
            std::cout << "Error RoadVisualizer type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["RoadVisualizer"]["calibration_params_path"].isNull() || root["RoadVisualizer"]["calibration_params_path"].type() != Json::objectValue || root["RoadVisualizer"]["calibration_params_path"]["cam_front"].isNull() || root["RoadVisualizer"]["calibration_params_path"]["cam_front"].type() != Json::stringValue || root["RoadVisualizer"]["calibration_params_path"]["cam_back"].isNull() || root["RoadVisualizer"]["calibration_params_path"]["cam_back"].type() != Json::stringValue)
        {
            std::cout << "Error calibration_params_path type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["RoadVisualizer"]["lane_list_topic"].isNull() || root["RoadVisualizer"]["lane_list_topic"].type() != Json::stringValue)
        {
            std::cout << "Error lane_list_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["RoadVisualizer"]["publish_topic"].isNull() || root["RoadVisualizer"]["publish_topic"].type() != Json::stringValue)
        {
            std::cout << "Error publish_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        cam_front = root["RoadVisualizer"]["calibration_params_path"]["cam_front"].asString();
        cam_back = root["RoadVisualizer"]["calibration_params_path"]["cam_back"].asString();
        lane_list_topic = root["RoadVisualizer"]["lane_list_topic"].asString();
        publish_topic = root["RoadVisualizer"]["publish_topic"].asString();
    }

    is.close();
    return true;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./RoadVisualizer <config_json_path> \n"
                     "example:\n"
                     "\t./bin/RoadVisualizer ./config/config.json"
                  << std::endl;
        return 1;
    }
    std::string cam_front, cam_back, lane_list_topic, publish_topic;
    if (!loadConfig(argv[1], cam_front, cam_back, lane_list_topic, publish_topic))
        return 1;

    ros::init(argc, argv, "RoadVisualizer");
    RoadVisualizer road_visualizer(
        cam_front,
        cam_back,
        lane_list_topic,
        publish_topic);
    road_visualizer.run();
    return 0;
}
