#pragma once
#include <vector>
#include <cstdlib>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "OpenNI.h"

#include "../../DWA/trans_module.h"
#include "constant.hpp"

typedef float Position[2];
/***************BACK_UP*************************
class AstraReader{
    public:
        AstraReader(trans_param astra2world);
        ~AstraReader();
        bool init();
        bool read_rgb(cv::Mat &img);
        bool read_depth(cv::Mat &img);
        bool depth2point(cv::Mat &depth_img, std::vector<cv::Point2f> &joints,
            std::vector<cv::Point3f> &points, std::vector<double> maxval);
        
        bool GenePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr AstraCloud);
        Position * depth2obs_astra(cv::Mat &depthframe , std::vector<cv::Point2f> &pixels , int * obs_nums ,const float obsheight ,const float obs_mindist = 0.05 ,const float valid_obs_dist = 3);
        
        cv::VideoCapture cap;
        openni::Device dev;
        openni::VideoStream depth_stream;
        // openni::VideoStream color_stream;
    
        double fov[2];
        double focus[2];
        double center[2];
        bool is_lidar;
        uint16_t max_depth;
        uint16_t min_depth; //相机内参

    private:
        float cos_theta;//用于转换到机器人坐标系
        float sin_theta;
        float Tx;
        float Ty;
};*/


class AstraReader{
    public:
        AstraReader(trans_param astra2world);
        ~AstraReader();
        void getDeviceUri();    // 双相机，需要通过每个相机的url打开设备
        bool init(int dev_index, int cap_index, int width, int hight);                   // 打开彩色流和深度流，设置深度图像和彩色图像的配准模式
        bool read_rgb(cv::Mat &img);
        bool read_depth(cv::Mat &img);
        bool depth2point(cv::Mat &depth_img, std::vector<cv::Point2f> &joints,
            std::vector<cv::Point3f> &points, std::vector<double> maxval);
        
        bool GenePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr AstraCloud);
        Position * depth2obs_astra(cv::Mat &depthframe , std::vector<cv::Point2f> &pixels , int * obs_nums ,const float obsheight ,const float obs_mindist = 0.05 ,const float valid_obs_dist = 3);
        
        static std::vector<std::string> deviceUrl_list;     // openni 的 device的uri 列表, 设置成静态变量，多个对象共享一份，初始化一次。
        cv::VideoCapture cap;
        openni::Device dev;
        openni::VideoStream depth_stream;
        // openni::VideoStream color_stream;
    
        double fov[2];
        double focus[2];
        double center[2];
        bool is_lidar;
        uint16_t max_depth;
        uint16_t min_depth; //相机内参

    private:
        float cos_theta;//用于转换到机器人坐标系
        float sin_theta;
        float Tx;
        float Ty;
};