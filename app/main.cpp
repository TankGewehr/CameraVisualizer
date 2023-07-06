#include "visualizer.h"

bool loadConfig(const std::string &filename,
                std::string &cam_front_left,
                std::string &cam_front,
                std::string &cam_front_right,
                std::string &cam_back_left,
                std::string &cam_back,
                std::string &cam_back_right,
                std::string &lidar_top,
                bool &obstacle_enable,
                std::string &obstacle_list_topic,
                bool &lane_enable,
                std::string &lane_list_topic,
                std::string &frame_id,
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
        if (root["calibration_params_path"].isNull() || root["calibration_params_path"].type() != Json::objectValue || root["calibration_params_path"]["cam_front_left"].isNull() || root["calibration_params_path"]["cam_front_left"].type() != Json::stringValue || root["calibration_params_path"]["cam_front"].isNull() || root["calibration_params_path"]["cam_front"].type() != Json::stringValue || root["calibration_params_path"]["cam_front_right"].isNull() || root["calibration_params_path"]["cam_front_right"].type() != Json::stringValue || root["calibration_params_path"]["cam_back_left"].isNull() || root["calibration_params_path"]["cam_back_left"].type() != Json::stringValue || root["calibration_params_path"]["cam_back"].isNull() || root["calibration_params_path"]["cam_back"].type() != Json::stringValue || root["calibration_params_path"]["cam_back_right"].isNull() || root["calibration_params_path"]["cam_back_right"].type() != Json::stringValue || root["calibration_params_path"]["lidar_top"].isNull() || root["calibration_params_path"]["lidar_top"].type() != Json::stringValue)
        {
            std::cout << "Error calibration_params_path type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["obstacle_enable"].isNull() || root["obstacle_enable"].type() != Json::booleanValue)
        {
            std::cout << "Error obstacle_enable type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["obstacle_list_topic"].isNull() || root["obstacle_list_topic"].type() != Json::stringValue)
        {
            std::cout << "Error obstacle_list_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["lane_enable"].isNull() || root["lane_enable"].type() != Json::booleanValue)
        {
            std::cout << "Error lane_enable type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["lane_list_topic"].isNull() || root["lane_list_topic"].type() != Json::stringValue)
        {
            std::cout << "Error lane_list_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["frame_id"].isNull() || root["frame_id"].type() != Json::stringValue)
        {
            std::cout << "Error frame_id type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["publish_topic"].isNull() || root["publish_topic"].type() != Json::stringValue)
        {
            std::cout << "Error publish_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        cam_front_left = root["calibration_params_path"]["cam_front_left"].asString();
        cam_front = root["calibration_params_path"]["cam_front"].asString();
        cam_front_right = root["calibration_params_path"]["cam_front_right"].asString();
        cam_back_left = root["calibration_params_path"]["cam_back_left"].asString();
        cam_back = root["calibration_params_path"]["cam_back"].asString();
        cam_back_right = root["calibration_params_path"]["cam_back_right"].asString();
        lidar_top = root["calibration_params_path"]["lidar_top"].asString();
        obstacle_enable = root["obstacle_enable"].asBool();
        obstacle_list_topic = root["obstacle_list_topic"].asString();
        lane_enable = root["lane_enable"].asBool();
        lane_list_topic = root["lane_list_topic"].asString();
        frame_id = root["frame_id"].asString();
        publish_topic = root["publish_topic"].asString();
    }

    is.close();
    return true;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./visualizer <config_json_path> \n"
                     "example:\n"
                     "\t./bin/visualizer ./config/config.json"
                  << std::endl;
        return 1;
    }
    std::string cam_front_left, cam_front, cam_front_right, cam_back_left, cam_back, cam_back_right, lidar_top, obstacle_list_topic, lane_list_topic, frame_id, publish_topic;
    bool obstacle_enable, lane_enable;
    if (!loadConfig(argv[1], cam_front_left, cam_front, cam_front_right, cam_back_left, cam_back, cam_back_right, lidar_top, obstacle_enable, obstacle_list_topic, lane_enable, lane_list_topic, frame_id, publish_topic))
        return 1;

    ros::init(argc, argv, "visualizer");
    visualizer app(
        cam_front_left,
        cam_front,
        cam_front_right,
        cam_back_left,
        cam_back,
        cam_back_right,
        lidar_top,
        obstacle_enable,
        obstacle_list_topic,
        lane_enable,
        lane_list_topic,
        frame_id,
        publish_topic);
    app.run();
    return 0;
}
