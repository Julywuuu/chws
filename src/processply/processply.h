// PCL lib Functions for processing point clouds 

#ifndef PROCESSPLY_H_
#define PROCESSPLY_H_

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>

#include <unordered_set>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>

#define WIDTH 640 
#define HEIGHT 480 
#define FPS 30




namespace obstacle_detection {

    // shorthand for point cloud pointer
    template<typename PointT>
    using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;

    template<typename PointT>
    struct CameraData{
        PtCdtr<PointT> pointcloud;
        cv::Mat color;
        cv::Mat depth;
    };

    struct CameraData2{
        cv::Mat color;
        cv::Mat depth;
    };

    template<typename PointT>

    class ProcessPointClouds {
    public:

        //constructor
        ProcessPointClouds();

        //deconstructor
        ~ProcessPointClouds();

        

        PtCdtr<PointT>PassThroughRemoveNoise(PtCdtr<PointT> cloud, float ymin=-0.6, float ymax=0.1);

        PtCdtr<PointT>RemoveGround(PtCdtr<PointT> cloud, float ymin=-0.4, float ymax=0.05);

        PtCdtr<PointT>BoundaryEstimate(PtCdtr<PointT> cloud);

        PtCdtr<PointT>Project2Ground(PtCdtr<PointT> cloud, float cam_height=0.5);

        PtCdtr<PointT>VGDownSampling(PtCdtr<PointT> cloud, float leftSize = 0.02f);

        PtCdtr<PointT> loadPly(std::string file);

        std::vector<boost::filesystem::path> streamPly(std::string dataPath);

        //启动相机
        rs2::pipeline startDepthCamera();

        //捕获相机深度帧、RGB帧，转换为pcl::PointCloud<PointT>::Ptr
        PtCdtr<PointT>Points2PCL(rs2::pipeline p);

        //捕获相机深度帧、RGB帧，转换为pcl::PointCloud<PointT>::Ptr以及opencv的Mat并返回
        std::pair<PtCdtr<PointT>, cv::Mat>Points2PCL_Color(rs2::pipeline p);

        //返回点云、深度帧、RGB帧
        CameraData<PointT> Points2PCL_Color_Depth(rs2::pipeline p);

        //返回深度帧、RGB帧
        CameraData2 Points2Color_Depth(rs2::pipeline p);

        //通过相机RGB帧的像素坐标，和该帧对应的点云，获取像素坐标对应的三维坐标
        float * get_3d_camera_coordinate(PtCdtr<PointT> cloud, int pixelx, int pixely);
        


    };
}
#endif /* PROCESSPLY_H_ */