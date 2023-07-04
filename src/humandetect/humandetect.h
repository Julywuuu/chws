#ifndef HUMANDETECT_H_
#define HUMANDETECT_H_

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/timeb.h>
#include <cmath>
#include <Eigen/Dense>

#include "constant.hpp"
#include "human_pose_estimator.hpp"
#include "astra_reader.hpp"
#include "intention_estimator.hpp"
#include "joints_writer.hpp"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
//#include "../intelreader/intel_reader.h"
#include "float.h"



namespace humandetect{

    //输入关节点位置(0-1表示，需要利用RGB图的大小转化为像素坐标)，返回对应点云坐标
    std::vector<cv::Point3f> get_joints_coord(std::vector<cv::Point2f> joints,std::vector<double> maxval,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    std::vector<cv::Point3f> get_joints_coord_bydepth(std::vector<cv::Point2f> joints,std::vector<double> maxval,
        cv::Mat depthframe);

    //输入关节点点云坐标，返回人体位置、朝向
    float * get_human_state(std::vector<cv::Point3f> joints_coord);

    float * get_human_state_knee(std::vector<cv::Point3f> joints_coord);

    //最小二乘拟合平面方程  z = a0x + a1y + a2
    std::vector<double> FittingPlaneZ(std::vector<cv::Point3f> peo_plane,int pixels_nums);

    //拟合髋关节附近人体平面，从而获取人体朝向
    float get_human_direction(std::vector<cv::Point2f> joints,std::vector<cv::Point3f> joints_coord,cv::Mat depthframe,std::vector<cv::Point2f> &pixels,int * pixels_nums);

    //获取一致的人体朝向，以此朝向为GT(暂时用不上)
    bool wait_consistent_direction(std::vector<cv::Point3f> joints_coord,float * consistent_direction);


}

#endif