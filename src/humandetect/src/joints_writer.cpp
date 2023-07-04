#include "joints_writer.hpp"

JointsWriter::JointsWriter()
{
    f.open(JOINTS_DATA_PATH, std::ios::out | std::ios::trunc);
}

JointsWriter::~JointsWriter()
{
    f.close();
}

bool JointsWriter::write(std::vector<cv::Point3f> joints, double t)
{
    if (f.is_open())
    {
        f << std::fixed << std::setprecision(6) << t;
        f << std::fixed << std::setprecision(3);
        for (int i = 0; i < joints.size(); i++)
        {
            f << "\t" << joints[i].x << "\t" << joints[i].y << "\t" << joints[i].z;
        }
        f << "\n";
        return true;
    }
    else
    {
        return false;
    }
    
    
}