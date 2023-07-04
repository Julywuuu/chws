#include "occupymapsimple.h"


bool OccupyMap::gridindex_in_blind(GridIndex index){
  std::vector<float> coord = ConvertGridIndex2World(index);
  return bc.obs_in_blind(coord);
}

//设置地图参数
void OccupyMap::SetMapParams(float x_range,float y_range,float resolution)
{
   //地图大小、分辨率
   mapParams.resolution = resolution;//0.04m 1/0.04=25个栅格

   mapParams.width = std::ceil(x_range/mapParams.resolution);
   mapParams.height = std::ceil(y_range/mapParams.resolution);     //单位栅格个数
   //假设free=-1、occ=2
   mapParams.log_free = -1;
   mapParams.log_occ = 2;
   //
   mapParams.origin_x = 0.0;
   mapParams.origin_y = 0.0;

   //地图的原点，即是机器人默认位置
   mapParams.offset_x = std::ceil(mapParams.width/2.0);
   mapParams.offset_y = std::ceil(mapParams.height/2.0);  //单位栅格个数
   //为地图指针申请空间
   pMap = new unsigned char[mapParams.width*mapParams.height];
   currentmap = new unsigned char[mapParams.width*mapParams.height];
   //每一个栅格代表的值，初始化为50
   for(int i = 0; i < mapParams.width * mapParams.height;i++){
      pMap[i] = 128;//128代表未知
   }
   //可视范围内的赋初值0，即视为已知且空闲,后续再用障碍物信息得到已知且占据（255）
   for(int i = 0; i < mapParams.width; i++){
      for(int j = 0;j < mapParams.height; j++){
        GridIndex tmpIndex;
        tmpIndex.SetIndex(i,j);
        if(!gridindex_in_blind(tmpIndex)){
          pMap[GridIndexToLinearIndex(tmpIndex)] = 0;
        }
      }
   }    

}

//从世界坐标系转换到栅格坐标系，主要是存在一个分辨率
//比如resolution = 0.04，世界坐标系下，单位1在栅格坐标系可以表示1/resolution=25个栅格
//目的：将机器人的实际位置，在900x900的栅格地图中找到对应的栅格序号，返回GridIndex对象
GridIndex OccupyMap::ConvertWorld2GridIndex(double x,double y)
{
    GridIndex index;
    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;
    return index;
}

//从栅格坐标系转换到世界坐标系
//返回栅格中心点的世界坐标
std::vector<float> OccupyMap::ConvertGridIndex2World(GridIndex index)
{
  std::vector<float> worldcoord(2);
  worldcoord[0] = (index.x - mapParams.offset_x)*mapParams.resolution + mapParams.origin_x;
  worldcoord[1] = -(index.y - mapParams.offset_y)*mapParams.resolution + mapParams.origin_y;;
  return worldcoord;
}

//从栅格序号，转化到数组序号；因为栅格最终是按照顺序（width从小到大，height从低到高）依次存储到动态数组中的
int OccupyMap::GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.x + index.y * mapParams.width;
}


//占据栅格地图构建算法
//输入障碍物位置指针和障碍物数目
//目的：输出栅格地图
void OccupyMap::generatemap(const float (*obs_position)[2],const int obs_nums)
{
    for(int i = 0; i < mapParams.width * mapParams.height;i++){
        currentmap[i] = pMap[i];//此时currentmap中只有盲区+可视范围，没有障碍物
    }
    //获取该帧机器人位姿的栅格序号
    GridIndex robotIndex = ConvertWorld2GridIndex(mapParams.origin_x,mapParams.origin_y);
    for(int id = 0; id < obs_nums;id++)
    {
      //明确这里的世界坐标系world_x，不是真实的世界坐标系，而是像素坐标系，y轴与真实的世界坐标系相反，这样是laser_y加负号的原因
      double laser_x =   obs_position[id][0];
      double laser_y =  -obs_position[id][1];

      //得到该激光扫描点，在世界坐标系下（像素坐标系下）的位置
      double world_x = laser_x;
      double world_y = laser_y;

      //将该激光扫描点在世界坐标系下的位置，转化为栅格序号
      GridIndex mapIndex = ConvertWorld2GridIndex(world_x,world_y);
      int tmpIndex = GridIndexToLinearIndex(mapIndex);
      currentmap[tmpIndex] = 255;
    }
}

//发布地图．
cv::Mat OccupyMap::PublishMap(void)
{
    // 创建一个空的灰度图像
    cv::Mat img(mapParams.height, mapParams.width, CV_8UC1, cv::Scalar(0));

    // 将二维数组中的数据复制到图像中
    for (int i = 0; i < mapParams.height; i++) {
        for (int j = 0; j < mapParams.width; j++) {
          img.at<uchar>(i, j) = currentmap[i*mapParams.width+j];
        }
    }
    cv::Mat rgbImg(img.size(),CV_8UC3);
		cv::cvtColor(img, rgbImg, cv::COLOR_GRAY2BGR);
    return rgbImg;
}

// int main(int argc, char** argv)
// {

//   char obsfile[] = "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/src/quiz/obscollect/obs.txt";
//   FILE* fp;
//   float x, y;
//   fp = fopen(obsfile, "r");  // 打开文件
//   if (fp == NULL) {
//       fprintf(stderr, "Error opening file %s\n", obsfile);
//   }
//   // 统计障碍物数量
//   int num_obstacles = 0;
//   while (fscanf(fp, "%f\t%f", &x, &y) != EOF) {//文件指针（FILE* 类型）指向的是一个文件流，每次读取或写入数据时，文件指针会自动向后移动到下一个位置
//       num_obstacles++;
//   }
//   std::vector<std::vector<float>> obs_position_temp;
//   // 读取坐标信息并存储到数组中
//   fseek(fp, 0, SEEK_SET);
//   int i = 0;
//   std::vector<float> obs(2);
//   while (fscanf(fp, "%f\t%f", &x, &y) != EOF) {
//       obs[0] = x;obs[1] = y;
//       obs_position_temp.push_back(obs);
//       i++;
//   }
//   float (*obs_position)[2] = new float[obs_position_temp.size()][2];
//   for(int i=0; i<obs_position_temp.size(); i++){
//       obs_position[i][0] = obs_position_temp[i][0];obs_position[i][1] = obs_position_temp[i][1];
//   }
//   fclose(fp);  // 关闭文件

//   for(int i = 0;i<obs_position_temp.size(); i++){
//       std::cout<< obs_position[i][0] <<"  " <<obs_position[i][1]<<std::endl;
//   }
//   float camera_height = 0.40;
//   float camera_height_back = 0.31;
//   std::vector<float> forwardcam{camera_height,58.4/180.0*M_PI,45.5/180.0*M_PI};
//   std::vector<float> backcam{camera_height_back,75.0/180.0*M_PI,62.0/180.0*M_PI};//相机的高度，水平视场角，垂直视场角
//   float obs_scan_height = 0.1;

//   BlindCompensate bc(6, forwardcam, backcam, obs_scan_height);


//   OccupyMap occum(bc);
//   //设置地图信息
//  // occum.SetMapParams();
//   //占用栅格地图构建算法
//   occum.generatemap(obs_position,obs_position_temp.size());


//   // //发布map，可视化
//   cv::Mat mapimage = occum.PublishMap();
//   // 显示图像
//   cv::namedWindow("Gray Image", cv::WINDOW_NORMAL);
//   cv::imshow("Gray Image", mapimage);
//   cv::waitKey(0);
//   // imwrite("/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/src/occupymap/obs.jpg", mapimage);
//   cv::destroyWindow("Gray Image");

//   std::cout <<"Release Memory!!"<<std::endl;


// }




