#include "CameraVisualizer.h"

bool loadConfig(const std::string &filename,
                std::string &cam_front_left,
                std::string &cam_front,
                std::string &cam_front_right,
                std::string &cam_back_left,
                std::string &cam_back,
                std::string &cam_back_right,
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
        if (root["CameraVisualizer"].isNull() || root["CameraVisualizer"].type() != Json::objectValue)
        {
            std::cout << "Error CameraVisualizer type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["CameraVisualizer"]["calibration_params_path"].isNull() || root["CameraVisualizer"]["calibration_params_path"].type() != Json::objectValue || root["CameraVisualizer"]["calibration_params_path"]["cam_front_left"].isNull() || root["CameraVisualizer"]["calibration_params_path"]["cam_front_left"].type() != Json::stringValue || root["CameraVisualizer"]["calibration_params_path"]["cam_front"].isNull() || root["CameraVisualizer"]["calibration_params_path"]["cam_front"].type() != Json::stringValue || root["CameraVisualizer"]["calibration_params_path"]["cam_front_right"].isNull() || root["CameraVisualizer"]["calibration_params_path"]["cam_front_right"].type() != Json::stringValue || root["CameraVisualizer"]["calibration_params_path"]["cam_back_left"].isNull() || root["CameraVisualizer"]["calibration_params_path"]["cam_back_left"].type() != Json::stringValue || root["CameraVisualizer"]["calibration_params_path"]["cam_back"].isNull() || root["CameraVisualizer"]["calibration_params_path"]["cam_back"].type() != Json::stringValue || root["CameraVisualizer"]["calibration_params_path"]["cam_back_right"].isNull() || root["CameraVisualizer"]["calibration_params_path"]["cam_back_right"].type() != Json::stringValue)
        {
            std::cout << "Error calibration_params_path type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["CameraVisualizer"]["obstacle_list_topic"].isNull() || root["CameraVisualizer"]["obstacle_list_topic"].type() != Json::stringValue)
        {
            std::cout << "Error obstacle_list_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["CameraVisualizer"]["publish_topic"].isNull() || root["CameraVisualizer"]["publish_topic"].type() != Json::stringValue)
        {
            std::cout << "Error publish_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        cam_front_left = root["CameraVisualizer"]["calibration_params_path"]["cam_front_left"].asString();
        cam_front = root["CameraVisualizer"]["calibration_params_path"]["cam_front"].asString();
        cam_front_right = root["CameraVisualizer"]["calibration_params_path"]["cam_front_right"].asString();
        cam_back_left = root["CameraVisualizer"]["calibration_params_path"]["cam_back_left"].asString();
        cam_back = root["CameraVisualizer"]["calibration_params_path"]["cam_back"].asString();
        cam_back_right = root["CameraVisualizer"]["calibration_params_path"]["cam_back_right"].asString();
        obstacle_list_topic = root["CameraVisualizer"]["obstacle_list_topic"].asString();
        publish_topic = root["CameraVisualizer"]["publish_topic"].asString();
    }

    is.close();
    return true;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./CameraVisualizerApp <config_json_path> \n"
                     "example:\n"
                     "\t./bin/CameraVisualizerApp ./config/config.json"
                  << std::endl;
        return 1;
    }
    std::string cam_front_left, cam_front, cam_front_right, cam_back_left, cam_back, cam_back_right, obstacle_list_topic, publish_topic;
    if (!loadConfig(argv[1], cam_front_left, cam_front, cam_front_right, cam_back_left, cam_back, cam_back_right, obstacle_list_topic, publish_topic))
        return 1;

    ros::init(argc, argv, "CameraVisualizer");
    CameraVisualizer *camera_visualizer = new CameraVisualizer(
        cam_front_left,
        cam_front,
        cam_front_right,
        cam_back_left,
        cam_back,
        cam_back_right,
        obstacle_list_topic,
        publish_topic);
    camera_visualizer->run();
    return 0;
}
