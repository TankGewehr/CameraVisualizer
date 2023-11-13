#include "sensors/Lidar.h"

Lidar::Lidar(std::string calibration_param_path)
{
    CalibrationParam calibration_param(calibration_param_path); // 标定参数

    this->channel = calibration_param.getChannel();
    this->extrinsic = calibration_param.getExtrinsic().inv();
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