// PCL lib Functions for processing point clouds 

#include "processply.h"

using namespace obstacle_detection;
using namespace cv;

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
PtCdtr<PointT>
ProcessPointClouds<PointT>::PassThroughRemoveNoise(PtCdtr<PointT> cloud, float ymin, float ymax){
    //auto startTime = std::chrono::steady_clock::now();

	PtCdtr<PointT> cloud_filted(new pcl::PointCloud<PointT>);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");																									//设置直通滤波器过滤字段方向
	pass.setFilterLimits(-3, -0.3);//取决于相机有效深度距离	
	pass.filter(*cloud_filted);
	pass.setInputCloud(cloud_filted);
    pass.setFilterFieldName("x");																									//设置直通滤波器过滤字段方向
	pass.setFilterLimits(-1.4, 1.4);//取决于相机有效深度距离及HFOV																							//设置过滤范围
	pass.filter(*cloud_filted);///cloud
	pass.setInputCloud(cloud_filted);
    pass.setFilterFieldName("y");																									//设置直通滤波器过滤字段方向
	pass.setFilterLimits(ymin,ymax);//初步去除不可能出现的点以及相机高度以上的点
	pass.filter(*cloud_filted);///cloud

	// auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "PassThroughRemoveNoise took " << elapsedTime.count() << " milliseconds" << std::endl;
	return cloud_filted;
}


template<typename PointT>
PtCdtr<PointT>
ProcessPointClouds<PointT>::RemoveGround(PtCdtr<PointT> cloud, float ymin, float ymax){
	//auto startTime = std::chrono::steady_clock::now();

	PtCdtr<PointT> cloud_filted(new pcl::PointCloud<PointT>);	
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(cloud);																										//设置直通滤波器输入点云
	pass.setFilterFieldName("y");																									//设置直通滤波器过滤字段方向
	pass.setFilterLimits(ymin, ymax);//去除地面，取决于相机安装高度
	pass.filter(*cloud_filted);
		
	// auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "RemoveGround took " << elapsedTime.count() << " milliseconds" << std::endl;
	return cloud_filted;
}

template<typename PointT>
PtCdtr<PointT>
ProcessPointClouds<PointT>::BoundaryEstimate(PtCdtr<PointT> cloud){
    auto startTime = std::chrono::steady_clock::now();

    PtCdtr<PointT> cloud_boundary(new pcl::PointCloud<PointT>);//初始化过滤后的点云
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	ne.setSearchMethod(tree);
	// Use all neighbors in a sphere of radius 1cm
	ne.setRadiusSearch(1);
	//ne.setKSearch(20);
	ne.compute(*normals);

	pcl::PointCloud<pcl::Boundary> boundary;
	pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> est;
	est.setInputCloud(cloud);
	est.setInputNormals(normals);
	est.setSearchMethod(tree);
	est.setKSearch(50); //一般这里的数值越高，最终边界识别的精度越好
	est.compute(boundary);

	for (int i = 0; i < cloud->points.size(); i++){
		if (boundary[i].boundary_point > 0){
			cloud_boundary->push_back(cloud->points[i]);
		}
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "BoundaryEstimate took " << elapsedTime.count() << " milliseconds" << std::endl;
	return cloud_boundary;
}

template<typename PointT>
PtCdtr<PointT>
ProcessPointClouds<PointT>::Project2Ground(PtCdtr<PointT> cloud, float cam_height){
	//auto startTime = std::chrono::steady_clock::now();

	PtCdtr<PointT> cloud_filted(new pcl::PointCloud<PointT>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	coefficients->values.resize (4);
	coefficients->values[0] = 0;
	coefficients->values[1] = 1.0;
	coefficients->values[2] = 0;
	coefficients->values[3] = cam_height;
	// Create the filtering object
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_filted);

	// auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "Project2Ground took " << elapsedTime.count() << " milliseconds" << std::endl;
    return cloud_filted;
}

template<typename PointT>
PtCdtr<PointT>
ProcessPointClouds<PointT>::VGDownSampling(PtCdtr<PointT> cloud, float leftSize){
	//auto startTime = std::chrono::steady_clock::now();

	PtCdtr<PointT> cloud_filted(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
	//float leftSize = 0.01f;//长宽高分别是1cm的体素过滤器
	vg.setInputCloud (cloud);
	vg.setLeafSize (leftSize, leftSize, leftSize);
	vg.filter (*cloud_filted);

	// auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "VGDownSampling took " << elapsedTime.count() << " milliseconds" << std::endl;
    return cloud_filted;
}



template<typename PointT>
PtCdtr<PointT> 
ProcessPointClouds<PointT>::loadPly(std::string file) {

    PtCdtr<PointT> cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPLYFile<PointT>(file, *cloud) == -1) { //* load the file
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> 
ProcessPointClouds<PointT>::streamPly(std::string dataPath) {

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
                                               boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template<typename PointT>
rs2::pipeline
ProcessPointClouds<PointT>::startDepthCamera(){
    // judge whether devices is exist or not 
	rs2::context ctx;
	auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
	if (list.size() == 0)
		throw std::runtime_error("无设备连接，请检查!");
	// Create a Pipeline - this serves as a top-level API for streaming and processing frames
	rs2::pipeline p;
	// Configure the pipeline
	rs2::config cfg;//创建一个以非默认配置的配置用来配置管道
	//Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);//向配置添加所需的流
	cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
	// Start the pipeline
	p.start(cfg);//指示管道使用所请求的配置启动流
    return p;
}

template<typename PointT>
PtCdtr<PointT>
ProcessPointClouds<PointT>::Points2PCL(rs2::pipeline p){
	rs2::points points;
	rs2::pointcloud pc;
	rs2::frameset frames = p.wait_for_frames();
	// Align to depth
	rs2::align align_to_depth(RS2_STREAM_COLOR);//声明与谁对齐-与深度流
	frames = align_to_depth.process(frames);//声明谁要对齐-采集获得的frames
	// Try to get a frame of a depth image
	rs2::video_frame depth = frames.get_depth_frame();
	// rs2::frame color = frames.get_color_frame();//不需要带颜色的点云
	// pc.map_to(color);
	points = pc.calculate(depth);

	//转化为pcl::PointCloud<PointT>::Ptr
	PtCdtr<PointT> cloud(new pcl::PointCloud<PointT>);
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = -ptr->y;//使得实时采集的点云坐标系与读取保存的ply文件的点云坐标系一致
        p.z = -ptr->z;
        ptr++;
    }

    return cloud;
}




template<typename PointT>
std::pair<PtCdtr<PointT>, cv::Mat>
ProcessPointClouds<PointT>::Points2PCL_Color(rs2::pipeline p){
	rs2::points points;
	rs2::pointcloud pc;
	rs2::frameset frames = p.wait_for_frames();
	// Align to depth
	rs2::align align_to_depth(RS2_STREAM_COLOR);//声明与谁对齐-与深度流
	frames = align_to_depth.process(frames);//声明谁要对齐-采集获得的frames
	// Try to get a frame of a depth image
	rs2::video_frame depth = frames.get_depth_frame();

	rs2::frame color_frame = frames.get_color_frame();
	cv::Mat color(Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
	//cv::Mat depth(Size(WIDTH, HEIGHT), CV_16UC1, (void*)depth.get_data(), Mat::AUTO_STEP);
	// pc.map_to(color);//不需要带颜色的点云
	points = pc.calculate(depth);

	//转化为pcl::PointCloud<PointT>::Ptr
	PtCdtr<PointT> cloud(new pcl::PointCloud<PointT>);
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = -ptr->y;//使得实时采集的点云坐标系与读取保存的ply文件的点云坐标系一致
        p.z = -ptr->z;
        ptr++;
    }
	
	std::pair<PtCdtr<PointT>, cv::Mat> PCL_Color(cloud,color);
	return PCL_Color;
}


//返回点云、深度帧、RGB帧
template<typename PointT>
CameraData<PointT>
ProcessPointClouds<PointT>::Points2PCL_Color_Depth(rs2::pipeline p){
	CameraData<PointT>cameradata;
	rs2::points points;
	rs2::pointcloud pc;
	rs2::frameset frames = p.wait_for_frames();
	// Align to depth
	rs2::align align_to_depth(RS2_STREAM_COLOR);//声明与谁对齐-与深度流
	frames = align_to_depth.process(frames);//声明谁要对齐-采集获得的frames
	// Try to get a frame of a depth image
	rs2::video_frame depth_frame = frames.get_depth_frame();

	rs2::frame color_frame = frames.get_color_frame();
	cv::Mat color(Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
	cv::Mat depth(Size(WIDTH, HEIGHT), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
	// pc.map_to(color);//不需要带颜色的点云
	points = pc.calculate(depth_frame);

	//转化为pcl::PointCloud<PointT>::Ptr
	PtCdtr<PointT> cloud(new pcl::PointCloud<PointT>);
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = -ptr->y;//使得实时采集的点云坐标系与读取保存的ply文件的点云坐标系一致
        p.z = -ptr->z;
        ptr++;
    }
	
	cameradata.pointcloud = cloud;
	cameradata.color = color;
	cameradata.depth = depth;
	return cameradata;
}


//返回深度帧、RGB帧
template<typename PointT>
CameraData2
ProcessPointClouds<PointT>::Points2Color_Depth(rs2::pipeline p){
	CameraData2 cameradata;
	rs2::frameset frames = p.wait_for_frames();
	// Align to depth
	rs2::align align_to_depth(RS2_STREAM_COLOR);//声明与谁对齐-与深度流
	frames = align_to_depth.process(frames);//声明谁要对齐-采集获得的frames
	// Try to get a frame of a depth image
	rs2::video_frame depth_frame = frames.get_depth_frame();

	rs2::frame color_frame = frames.get_color_frame();
	cv::Mat color(Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
	cv::Mat depth(Size(WIDTH, HEIGHT), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

	cameradata.color = color;
	cameradata.depth = depth;
	return cameradata;
}


template<typename PointT>
float * 
ProcessPointClouds<PointT>::get_3d_camera_coordinate(PtCdtr<PointT> cloud, int pixelx, int pixely){//cloud坐标系已经由Points2PCL_Color转换过,这里不用再转换
	int index = pixely*WIDTH + pixelx;
	float * coord = new float[3];
	coord[0] = cloud->points[index].x;
    coord[1] = cloud->points[index].y;
	coord[2] = cloud->points[index].z;
	return coord;
}
