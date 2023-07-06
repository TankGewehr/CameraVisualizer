#include "sensors/Lidar.h"

Lidar::Lidar(std::string intrinsic_and_extrinsic_json_path)
{
    Json::Reader reader;
    Json::Value root;

    std::ifstream is(intrinsic_and_extrinsic_json_path, std::ios::binary);
    if (!is.is_open())
    {
        std::cout << "Error opening file:" << intrinsic_and_extrinsic_json_path << std::endl;
    }
    else
    {
        if (reader.parse(is, root))
        {
            if (!root["channel"].isNull() && root["channel"].type() == Json::stringValue)
            {
                this->channel = root["channel"].asString();
            }
            else
            {
                std::cout << "Error channel type:" << intrinsic_and_extrinsic_json_path << std::endl;
            }
        }

        is.close();
    }

    cv::Mat extrinsic;
    loadExtrinsic(intrinsic_and_extrinsic_json_path, extrinsic);
    this->extrinsic = extrinsic.inv();
}

Lidar::~Lidar() = default;

void Lidar::setFrameId(std::string frame_id)
{
    this->frame_id = frame_id;
}

std::string Lidar::getChannel() const
{
    return this->channel;
}

std::string Lidar::getFrameId() const
{
    return this->frame_id;
}

cv::Mat Lidar::getExtrinsic() const
{
    return this->extrinsic;
}