#pragma once

#include <opencv2/opencv.hpp>
#include "calibration_params.h"

class Lidar
{
private:
    std::string channel;
    std::string frame_id;

    cv::Mat extrinsic;

public:
    Lidar(std::string intrinsic_and_extrinsic_json_path);
    ~Lidar();

    void setFrameId(std::string frame_id);

    std::string getChannel() const;

    std::string getFrameId() const;

    cv::Mat getExtrinsic() const;
};