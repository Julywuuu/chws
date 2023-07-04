#include "occupymap.h"


bool OccupyMap::gridindex_in_blind(GridIndex index){
  return true;
}

//2D画线算法　来进行计算两个点之间的grid cell
//目的：找到两点之间途径的空闲栅格，将栅格序号存入std::vector<GridIndex>中
std::vector<GridIndex> OccupyMap::TraceLine(int x0, int y0, int x1, int y1)
{
  GridIndex tmpIndex;
  std::vector<GridIndex> gridIndexVector;

  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep)
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1)
  {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int deltaX = x1 - x0;
  int deltaY = abs(y1 - y0);
  int error = 0;
  int ystep;
  int y = y0;

  if (y0 < y1)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }

  int pointX;
  int pointY;
  for (int x = x0; x <= x1; x++)
  {
    if (steep)
    {
      pointX = y;
      pointY = x;
    }
    else
    {
      pointX = x;
      pointY = y;
    }

    error += deltaY;

    if (2 * error >= deltaX)
    {
      y += ystep;
      error -= deltaX;
    }

    //不包含最后一个点．
    if(pointX == x1 && pointY == y1) continue;

    //保存所有的点
    tmpIndex.SetIndex(pointX,pointY);

    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
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

   //每一个栅格代表的值，初始化为50
   for(int i = 0; i < mapParams.width * mapParams.height;i++){
      pMap[i] = 50;//50代表未知
   }
   //可视范围内的赋初值50+log_free=49，即视为已知且空闲
   for(int i = 0; i < mapParams.width; i++){
      for(int j = 0;j < mapParams.height; j++){
        GridIndex tmpIndex;
        tmpIndex.SetIndex(i,j);
        if(!gridindex_in_blind(tmpIndex)){
          pMap[GridIndexToLinearIndex(tmpIndex)] += mapParams.log_free;
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
    //ceil()向上取整函数
    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;

    return index;
}

//从栅格序号，转化到数组序号；因为栅格最终是按照顺序（width从小到大，height从低到高）依次存储到动态数组中的
int OccupyMap::GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.x + index.y * mapParams.width;
}

//判断index是否有效
//目的：判断该栅格序号是否在设定栅格地图大小范围内
bool OccupyMap::isValidGridIndex(GridIndex index)
{
    if(index.x >= 0 && index.x < mapParams.width && index.y >= 0 && index.y < mapParams.height)
        return true;

    return false;
}

//占据栅格地图构建算法
//输入激光雷达数据和机器人位姿数据
//目的：通过遍历所有帧数据，为pMap[]中的每个穿过的空闲栅格或者击中栅格赋新值，中间有个计算方法，也就是占用栅格地图构建的理论实现
void OccupyMap::OccupanyMapping(std::vector<GeneralLaserScan>& scans,std::vector<Eigen::Vector3d>& robot_poses)
{
  std::cout <<"Scans Size:"<<scans.size()<<std::endl;
  std::cout <<"Poses Size:"<<robot_poses.size()<<std::endl;

  //遍历所有帧激光雷达数据
  for(int i = 0; i < scans.size();i++)
  {
    //获取每一帧的激光雷达、机器人位姿数据
    GeneralLaserScan scan = scans[i];
    Eigen::Vector3d robotPose = robot_poses[i];

    //获取该帧机器人位姿的栅格序号
    GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0),robotPose(1));

    //判断该帧机器人位姿的栅格序号，是否在自己设定的栅格地图范围内
    if(isValidGridIndex(robotIndex) == false) continue;
  
    //遍历该帧激光雷达数据所有扫描点
    for(int id = 0; id < scan.range_readings.size();id++)
    {
      //取出该激光雷达扫描点的距离和角度
      double dist = scan.range_readings[id];
      double angle = scan.angle_readings[id];
      //剔除异常数据，跳过该次循环，不作处理
      if(std::isinf(dist) || std::isnan(dist)) continue;
      //机器人航向角，机器人x轴与世界坐标系x轴夹角
      double theta = robotPose(2);

      //在旋转过后（与世界坐标系（像素坐标系下）平行）的激光雷达坐标系下的坐标x,y
      //该开始一直不理解这个为啥laser_y要加一个负号
      //明确激光雷达数据的角度逆时针变化
      //明确机器人航向角与世界坐标系x轴呈逆时针变化
      //明确这里的世界坐标系world_x，不是真实的世界坐标系，而是像素坐标系，y轴与真实的世界坐标系相反，这样是laser_y加负号的原因
      double laser_x =   dist * cos(theta + angle);
      double laser_y =  -dist * sin(theta + angle);

      //得到该激光扫描点，在世界坐标系下（像素坐标系下）的位置
      double world_x = laser_x + robotPose(0);
      double world_y = laser_y + robotPose(1);

      //将该激光扫描点在世界坐标系下的位置，转化为栅格序号
      GridIndex mapIndex = ConvertWorld2GridIndex(world_x,world_y);

      //判断该激光扫描点的栅格序号，是否在自己设定的栅格地图900x900范围内，如果不在则跳过
      if(isValidGridIndex(mapIndex) == false)continue;

      //从机器人的栅格序号到该激光扫描点的栅格序号划线
      //目的：找到两点之间途径的空闲栅格，将栅格序号存入std::vector<GridIndex>中
      std::vector<GridIndex> freeIndex = TraceLine(robotIndex.x,robotIndex.y,mapIndex.x,mapIndex.y);

      //遍历该扫描激光点通过的所有空闲栅格
      for(int k = 0; k < freeIndex.size();k++)
      {
        GridIndex tmpIndex = freeIndex[k];
        //将空闲栅格的栅格序号，转化到数组序号,该数组用于存储每一个栅格的数据
        int linearIndex = GridIndexToLinearIndex(tmpIndex);
        //取出该栅格代表的数据
        int data = pMap[linearIndex];
        //根据栅格空闲规则，执行data += mapParams.log_free;
        if(data > 0)//默认data=50
          data += mapParams.log_free;//log_free=-1，data将变小
        else
          data = 0;
        //给该空闲栅格赋新值，最小为0
        pMap[linearIndex] = data;

      }

      //更新该激光扫描点集中的栅格，
      int tmpIndex = GridIndexToLinearIndex(mapIndex);
      int data = pMap[tmpIndex];
      //根据栅格击中规则，执行data += mapParams.log_occ;
      if(data < 100)//默认data=50
        data += mapParams.log_occ;//log_occ=2，data将变大
      else
        data = 100;
      //给击中的栅格赋新值，最大100
      pMap[tmpIndex] = data;
      //到这里，对一个位姿下的一个激光扫描数据经过的空闲栅格和击中栅格的pMap进行了重新赋值
    }
    //到这里，对一个位姿下的一帧激光扫描数据经过的空闲栅格和击中栅格进行了重新赋值
  }
  //到这里，对所有帧激光扫描数据经过的空闲栅格和击中栅格进行了重新赋值
}


//占据栅格地图构建算法
//输入障碍物位置指针和障碍物数目
//目的：输出栅格地图
void OccupyMap::generatemap(const float (*obs_position)[2],const int obs_nums)
{
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

      //判断该激光扫描点的栅格序号，是否在自己设定的栅格地图900x900范围内，如果不在则跳过
      if(isValidGridIndex(mapIndex) == false)continue;

      //从机器人的栅格序号到该激光扫描点的栅格序号划线
      //目的：找到两点之间途径的空闲栅格，将栅格序号存入std::vector<GridIndex>中
      std::vector<GridIndex> freeIndex = TraceLine(robotIndex.x,robotIndex.y,mapIndex.x,mapIndex.y);

      //遍历该扫描激光点通过的所有空闲栅格
      for(int k = 0; k < freeIndex.size();k++)
      {
        GridIndex tmpIndex = freeIndex[k];
        //将空闲栅格的栅格序号，转化到数组序号,该数组用于存储每一个栅格的数据
        int linearIndex = GridIndexToLinearIndex(tmpIndex);
        //取出该栅格代表的数据
        int data = pMap[linearIndex];
        //根据栅格空闲规则，执行data += mapParams.log_free;
        if(data > 0)//默认data=50
          data += mapParams.log_free;//log_free=-1，data将变小
        else
          data = 0;
        //给该空闲栅格赋新值，最小为0
        pMap[linearIndex] = data;

      }

      //更新该激光扫描点集中的栅格，
      int tmpIndex = GridIndexToLinearIndex(mapIndex);
      int data = pMap[tmpIndex];
      //根据栅格击中规则，执行data += mapParams.log_occ;
      if(data < 100)//默认data=50
        data += mapParams.log_occ;//log_occ=2，data将变大
      else
        data = 100;
      //给击中的栅格赋新值，最大100
      pMap[tmpIndex] = data;
      //到这里，对一个位姿下的一个激光扫描数据经过的空闲栅格和击中栅格的pMap进行了重新赋值
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
            if(pMap[i*mapParams.width+j] == 50)    //未知栅格
            {
                img.at<uchar>(i, j) = 128;
            }
            else if(pMap[i*mapParams.width+j] < 50)//空闲栅格 vb
            {
               img.at<uchar>(i, j) = 0;   //gmapping    方式
                // img.at<uchar>(i, j) = pMap[i*mapParams.width+j];//cartographer方式
            }
            else if(pMap[i*mapParams.width+j] > 50)//击中栅格
            {
               img.at<uchar>(i, j) = 255;
                // img.at<uchar>(i, j) = pMap[i*mapParams.width+j];
            }
        }
    }
    return img;
}

int main(int argc, char** argv)
{
  // std::vector<Eigen::Vector3d> robotPoses;
  // std::vector<GeneralLaserScan> generalLaserScans;

  // std::string basePath = "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/src/quiz/occupany_mapping/data";

  // std::string posePath= basePath + "/pose.txt";
  // std::string anglePath = basePath + "/scanAngles.txt";
  // std::string scanPath = basePath + "/ranges.txt";

  // //读取机器人位姿数据
  // ReadPoseInformation(posePath,robotPoses);
  // //读取激光雷达数据
  // ReadLaserScanInformation(anglePath,
  //                           scanPath,
  //                           generalLaserScans);

  char obsfile[] = "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/src/quiz/obscollect/obs.txt";
  FILE* fp;
  float x, y;
  fp = fopen(obsfile, "r");  // 打开文件
  if (fp == NULL) {
      fprintf(stderr, "Error opening file %s\n", obsfile);
  }
  // 统计障碍物数量
  int num_obstacles = 0;
  while (fscanf(fp, "%f\t%f", &x, &y) != EOF) {//文件指针（FILE* 类型）指向的是一个文件流，每次读取或写入数据时，文件指针会自动向后移动到下一个位置
      num_obstacles++;
  }
  std::vector<std::vector<float>> obs_position_temp;
  // 读取坐标信息并存储到数组中
  fseek(fp, 0, SEEK_SET);
  int i = 0;
  std::vector<float> obs(2);
  while (fscanf(fp, "%f\t%f", &x, &y) != EOF) {
      obs[0] = x;obs[1] = y;
      obs_position_temp.push_back(obs);
      i++;
  }
  float (*obs_position)[2] = new float[obs_position_temp.size()][2];
  for(int i=0; i<obs_position_temp.size(); i++){
      obs_position[i][0] = obs_position_temp[i][0];obs_position[i][1] = obs_position_temp[i][1];
  }
  fclose(fp);  // 关闭文件

  for(int i = 0;i<obs_position_temp.size(); i++){
      std::cout<< obs_position[i][0] <<"  " <<obs_position[i][1]<<std::endl;
  }
  float camera_height = 0.40;
  float camera_height_back = 0.31;
  std::vector<float> forwardcam{camera_height,58.4/180.0*M_PI,45.5/180.0*M_PI};
  std::vector<float> backcam{camera_height_back,75.0/180.0*M_PI,62.0/180.0*M_PI};//相机的高度，水平视场角，垂直视场角
  float obs_scan_height = 0.1;

  OccupyMap occum(forwardcam,backcam,obs_scan_height);
  //设置地图信息
  occum.SetMapParams();
  //占用栅格地图构建算法
  occum.generatemap(obs_position,obs_position_temp.size());
  // occum.OccupanyMapping(generalLaserScans,robotPoses);
  // //发布map，可视化
  cv::Mat mapimage = occum.PublishMap();
  // 显示图像
  cv::namedWindow("Gray Image", cv::WINDOW_NORMAL);
  cv::imshow("Gray Image", mapimage);
  cv::waitKey(0);
  cv::destroyWindow("Gray Image");

  std::cout <<"Release Memory!!"<<std::endl;

}




