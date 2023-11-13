#pragma once

#include <opencv2/opencv.hpp>
#include "CalibrationParam.h"

class Lidar
{
private:
    std::string channel;
    std::string frame_id;

    cv::Mat extrinsic;

public:
    Lidar(std::string calibration_param_path);
    ~Lidar();

    void setFrameId(std::string frame_id);

    std::string getChannel() const;

    std::string getFrameId() const;

    cv::Mat getExtrinsic() const;
};