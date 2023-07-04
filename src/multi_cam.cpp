// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <string>
#include <sstream>              // Stringstreams
#include <fstream>              // File IO
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>

#include <Eigen/Dense>
#include "humandetect/humandetect.h"
#include "intelreader/intel_reader.h"
#include "conio/conio.h"
#include "OpenNI.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;
using namespace cv;

#define WIDTH 640 
#define HEIGHT 480 
#define FPS 30

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, float r = 1,float g = 1,float b = 1,float pointsize = 2)
{
  	viewer->addPointCloud<pcl::PointXYZ> (cloud, name);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointsize, name);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name);
}


int main(int argc, char * argv[]) try
{	
	/// 方式1：Matrix4f
	// 创建矩阵对象transform_1，初始化为4×4单位阵
	// Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	// // 定义旋转矩阵，绕z轴
	// float theta = M_PI / 4;		// 旋转弧度
	// transform_1(0, 0) = cos(theta);
	// transform_1(0, 1) = -sin(theta);
	// transform_1(1, 0) = sin(theta);
	// transform_1(1, 1) = cos(theta);
	// // 定义在x轴上的平移，2.5m
	// transform_1(0, 3) = 2.5;
	// // 打印平移、旋转矩阵
	// std::cout << "方式1: 使用Matrix4f\n";
	// std::cout << transform_1 << std::endl;

	/// 方式2：Affine3f
	// 创建矩阵对象transform_2.matrix()，初始化为4×4单位阵
	// float theta = M_PI;		// 旋转弧度
	// Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	// // 定义在轴上的平移
	// transform_2.translation() << 0.045, 0.00, 0.085;	// 三个数分别对应X轴、Y轴、Z轴方向上的平移
	// // 定义旋转矩阵，绕y轴
	// transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));	//同理，UnitX(),绕X轴；UnitY(),绕Y轴.
	// // 打印平移、旋转矩阵
	// std::cout << "\n方式2: 使用Affine3f\n";
	// std::cout << transform_2.matrix() << std::endl;	//注意：不是transform_2

	timeb t_start, t_intel_color, t_intel_capture1, t_intel_capture2, t_astra_capture;

	// /*启动intel相机*/
	IntelReader intel_reader;
	rs2::points points;
	rs2::pointcloud pc;
	rs2::align align_to_color(RS2_STREAM_COLOR);//声明与谁对齐-与颜色流
	if (!intel_reader.init()){
		std::cout << "Intel相机未连接，请检查!" << std::endl;
	}
	std::cout<<"相机1已启动..."<<std::endl;

	//sleep(10);
	/*启动astra相机*/
	AstraReader astra_reader;
	cv::Mat astra_rgb;
	if (!astra_reader.init()){
		std::cout << "Astra相机未连接，请检查!" << std::endl;
	}
    std::cout<<"相机2已启动..."<<std::endl;
    


	//pcl::PointCloud<pcl::PointXYZ>::Ptr D435cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr D435cloud_SDK(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Astracloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Astracloud_in_D435coord(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	//while (true){
	while (!viewer->wasStopped())
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
		// ftime(&t_start);

        // //读取Intel相机数据
		// rs2::frameset frames = intel_reader.pipeline.wait_for_frames();
		// frames = align_to_color.process(frames);//声明谁要对齐-采集获得的frames
		// rs2::video_frame depth = frames.get_depth_frame();
		// rs2::frame color = frames.get_color_frame();
        // Mat colorimage(Size(depth.get_width(), depth.get_height()), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
		// ftime(&t_intel_color);
		// double temp = t_intel_color.time + t_intel_color.millitm * 1e-3 - (t_start.time + t_start.millitm * 1e-3);

		// points = pc.calculate(depth);
		// intel_reader.rs2pcl(points,D435cloud_SDK);

		// ftime(&t_intel_capture1);
		// cout << "英特尔相机SDK转PCL+RGB捕获耗时intel_time1 =  " <<dec<< (t_intel_capture1.time + t_intel_capture1.millitm * 1e-3 - (t_intel_color.time + t_intel_color.millitm * 1e-3) + temp)*1000 << endl;

		// intel_reader.GenePointcloud(D435cloud,depth);
		// ftime(&t_intel_capture2);
		// cout << "英特尔相机直接PCL+RGB捕获耗时intel_time2 =  " <<dec<< (t_intel_capture2.time + t_intel_capture2.millitm * 1e-3 - (t_intel_capture1.time + t_intel_capture1.millitm * 1e-3) + temp)*1000 << endl;

        //读取Astra相机数据
		astra_reader.GenePointcloud(Astracloud);
		// astra_reader.read_rgb(astra_rgb);
		// ftime(&t_astra_capture);
		// double cap_time = t_astra_capture.time + t_astra_capture.millitm * 1e-3 - (t_intel_capture2.time + t_intel_capture2.millitm * 1e-3);
		// cout << "奥比中光相机直接PCL+RGB捕获耗时astra_time =  " <<dec<< cap_time*1000 << endl;
		//对于整个点云这个函数很慢，考虑先降采样后得到障碍物坐标再参考该函数转换
		// pcl::transformPointCloud(*Astracloud, *Astracloud_in_D435coord, transform_2);

		// cv::circle(astra_rgb, cv::Point(320, 240), 3, cv::Scalar(255, 0, 255), 1);
		// cv::circle(astra_rgb, cv::Point(480, 120), 3, cv::Scalar(255, 0, 255), 1);
		// cv::circle(astra_rgb, cv::Point(160, 120), 3, cv::Scalar(255, 0, 255), 1);
		// cv::circle(astra_rgb, cv::Point(160, 360), 3, cv::Scalar(255, 0, 255), 1);
		// cv::circle(astra_rgb, cv::Point(480, 360), 3, cv::Scalar(255, 0, 255), 1);
		// cv::circle(colorimage, cv::Point(320, 240), 3, cv::Scalar(255, 0, 255), 1);
		// cv::circle(colorimage, cv::Point(480, 120), 3, cv::Scalar(255, 0, 255), 1);
		// cv::circle(colorimage, cv::Point(160, 120), 3, cv::Scalar(255, 0, 255), 1);
		// cv::circle(colorimage, cv::Point(160, 360), 3, cv::Scalar(255, 0, 255), 1);
		// cv::circle(colorimage, cv::Point(480, 360), 3, cv::Scalar(255, 0, 255), 1);

		// std::cout <<"  中心点坐标 = [  "<<Astracloud->points[240*WIDTH + 320].x<< " , "<<Astracloud->points[240*WIDTH + 320].y<<" , "<<Astracloud->points[240*WIDTH + 320].z<<"  ]"<<std::endl;
		// std::cout <<"第一象限坐标 = [  "<<Astracloud->points[120*WIDTH + 480].x<< " , "<<Astracloud->points[120*WIDTH + 480].y<<" , "<<Astracloud->points[120*WIDTH + 480].z<<"  ]"<<std::endl;
		// std::cout <<"第二象限坐标 = [  "<<Astracloud->points[120*WIDTH + 160].x<< " , "<<Astracloud->points[120*WIDTH + 160].y<<" , "<<Astracloud->points[120*WIDTH + 160].z<<"  ]"<<std::endl;
		// std::cout <<"第三象限坐标 = [  "<<Astracloud->points[360*WIDTH + 160].x<< " , "<<Astracloud->points[360*WIDTH + 160].y<<" , "<<Astracloud->points[360*WIDTH + 160].z<<"  ]"<<std::endl;
		// std::cout <<"第四象限坐标 = [  "<<Astracloud->points[360*WIDTH + 480].x<< " , "<<Astracloud->points[360*WIDTH + 480].y<<" , "<<Astracloud->points[360*WIDTH + 480].z<<"  ]"<<std::endl;
		// std::cout << "**********************" << std::endl;
		// std::cout <<"  中心点坐标 = [  "<<D435cloud->points[240*WIDTH + 320].x<< " , "<<D435cloud->points[240*WIDTH + 320].y<<" , "<<D435cloud->points[240*WIDTH + 320].z<<"  ]"<<std::endl;
		// std::cout <<"第一象限坐标 = [  "<<D435cloud->points[120*WIDTH + 480].x<< " , "<<D435cloud->points[120*WIDTH + 480].y<<" , "<<D435cloud->points[120*WIDTH + 480].z<<"  ]"<<std::endl;
		// std::cout <<"第二象限坐标 = [  "<<D435cloud->points[120*WIDTH + 160].x<< " , "<<D435cloud->points[120*WIDTH + 160].y<<" , "<<D435cloud->points[120*WIDTH + 160].z<<"  ]"<<std::endl;
		// std::cout <<"第三象限坐标 = [  "<<D435cloud->points[360*WIDTH + 160].x<< " , "<<D435cloud->points[360*WIDTH + 160].y<<" , "<<D435cloud->points[360*WIDTH + 160].z<<"  ]"<<std::endl;
		// std::cout <<"第四象限坐标 = [  "<<D435cloud->points[360*WIDTH + 480].x<< " , "<<D435cloud->points[360*WIDTH + 480].y<<" , "<<D435cloud->points[360*WIDTH + 480].z<<"  ]"<<std::endl;
		// //renderPointCloud(viewer, Astracloud, "Astracloud", 2);
		// std::cout <<"  中心点坐标差 = [  "<<D435cloud->points[240*WIDTH + 320].x - D435cloud_SDK->points[240*WIDTH + 320].x<< " , "<<D435cloud->points[240*WIDTH + 320].y - D435cloud_SDK->points[240*WIDTH + 320].y<<" , "<<D435cloud->points[240*WIDTH + 320].z - D435cloud_SDK->points[240*WIDTH + 320].z<<"  ]"<<std::endl;
		// std::cout <<"第一象限坐标差 = [  "<<D435cloud->points[120*WIDTH + 480].x - D435cloud_SDK->points[120*WIDTH + 480].x<< " , "<<D435cloud->points[120*WIDTH + 480].y - D435cloud_SDK->points[120*WIDTH + 480].y<<" , "<<D435cloud->points[120*WIDTH + 480].z - D435cloud_SDK->points[120*WIDTH + 480].z<<"  ]"<<std::endl;
		// std::cout <<"第二象限坐标差 = [  "<<D435cloud->points[120*WIDTH + 160].x - D435cloud_SDK->points[120*WIDTH + 160].x<< " , "<<D435cloud->points[120*WIDTH + 160].y - D435cloud_SDK->points[120*WIDTH + 160].y<<" , "<<D435cloud->points[120*WIDTH + 160].z - D435cloud_SDK->points[120*WIDTH + 160].z<<"  ]"<<std::endl;
		// std::cout <<"第三象限坐标差 = [  "<<D435cloud->points[360*WIDTH + 160].x - D435cloud_SDK->points[360*WIDTH + 160].x<< " , "<<D435cloud->points[360*WIDTH + 160].y - D435cloud_SDK->points[360*WIDTH + 160].y<<" , "<<D435cloud->points[360*WIDTH + 160].z - D435cloud_SDK->points[360*WIDTH + 160].z<<"  ]"<<std::endl;
		// std::cout <<"第四象限坐标差 = [  "<<D435cloud->points[360*WIDTH + 480].x - D435cloud_SDK->points[360*WIDTH + 480].x<< " , "<<D435cloud->points[360*WIDTH + 480].y - D435cloud_SDK->points[360*WIDTH + 480].y<<" , "<<D435cloud->points[360*WIDTH + 480].z - D435cloud_SDK->points[360*WIDTH + 480].z<<"  ]"<<std::endl;

		// renderPointCloud(viewer, D435cloud_SDK, "D435cloud_SDK", 1,0,0,0.1);
		// renderPointCloud(viewer, Astracloud_in_D435coord, "Astracloud_in_D435coord", 0,1,0,0.1);
		renderPointCloud(viewer, Astracloud, "Astracloud", 0,1,0,0.1);

		
		
        // Mat des;
		// des.create(colorimage.rows, colorimage.cols + astra_rgb.cols, colorimage.type());
		// Mat area = des(Rect(0, 0, colorimage.cols, colorimage.rows));
		// colorimage.copyTo(area);
		// Mat area1 = des(Rect(colorimage.cols, 0, colorimage.cols, colorimage.rows));
		// astra_rgb.copyTo(area1);
        // namedWindow("Display Image", WINDOW_AUTOSIZE);
        // imshow("Display Image", des);

		//imshow("Display Image", astra_rgb);
		viewer->spinOnce();
		//int key = waitKey(1);
		// if (key == 27)
		// 	break;
		if(kbhit()) {
            int key = getch();
        	if(key == KEY_ESC) {//按ESC键退出
                break;
        	}
		}
	}
	std::cout<<"退出成功"<<std::endl;
	return EXIT_SUCCESS;
}


catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
