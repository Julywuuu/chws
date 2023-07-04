#pragma once
#include <fstream>
#include <sstream>
#include <cstring>
#include <iomanip>
#include <vector>
#include <opencv2/opencv.hpp>

#include "constant.hpp"

class JointsWriter
{
public:
    JointsWriter();
    ~JointsWriter();
    bool write(std::vector<cv::Point3f> joints, double t);
    std::fstream f;
};