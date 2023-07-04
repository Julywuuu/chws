#ifndef OCCUPYMAP_H_
#define OCCUPYMAP_H_

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/src/quiz/occupany_mapping/include/occupany_mapping/readfile.h"

typedef float Position[2];
//自定义名为GridIndex（栅格序号）的数据类型
//目的：每一个GridIndex，包含x,y坐标
typedef struct gridindex_
{
    int x;
    int y;

    void SetIndex(int x_,int y_)
    {
        x  = x_;
        y  = y_;
    }
}GridIndex;

//自定义名为MapParams（地图参数）的数据类型
//目的：包含地图的最基本信息；分辨率、地图原点、地图大小、偏移等等
typedef struct map_params
{
    double log_occ,log_free;
    double resolution;
    double origin_x,origin_y;
    int height,width;
    int offset_x,offset_y;
}MapParams;


class OccupyMap{
    private:
        std::vector<float> _forwardcam;//存储前面相机的高度，水平视场角，垂直视场角
        std::vector<float> _backcam;
        float _obs_scan_height;
        // float valid_ymax = 1.5;//地图有效范围
        // float valid_ymin = -1.5;
        // float valid_xmax = 3.0;
        // float valid_xmin = -3.0; 
        // float robotmodel_radius = 0.4;//待小车障碍物补偿机制健全后可设为小车模型大小0.35
        // float human_radius = 0.3;//人体位置附近的判定距离
        float forwardblind_y;//垂直视场角形成的盲区
        float backblind_y;
    public:
        MapParams mapParams;
        //地图指针
        unsigned char* pMap;

        OccupyMap(std::vector<float> forwardcam,std::vector<float> backcam, float obs_scan_height) :
         _forwardcam(forwardcam),
         _backcam(backcam),
         _obs_scan_height(obs_scan_height){
            //减_obs_scan_height得到障碍物所在平面的盲区，减0.02使盲区判定更严格，避免相机波动获取的障碍物经盲区补偿后投射到当前帧盲区边缘，被判定有效
            forwardblind_y = (_forwardcam[0]-_obs_scan_height-0.02)/tan(_forwardcam[2]/2.0);
            backblind_y = -(_backcam[0]-_obs_scan_height+0.09)/tan(_backcam[2]/2.0);//使得backblind_y=-0.5左右，增大后向盲区范围用于补偿
        };

        ~OccupyMap(){    
            if(pMap != NULL)
                delete pMap;
        };

        //判断栅格是否在盲区
        bool gridindex_in_blind(GridIndex index);

        std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1);
        void SetMapParams(float x_range = 6.0,float y_range = 3.0,float resolution = 0.04);
        GridIndex ConvertWorld2GridIndex(double x,double y);
        int GridIndexToLinearIndex(GridIndex index);
        bool isValidGridIndex(GridIndex index);
        // void DestoryMap();
        void OccupanyMapping(std::vector<GeneralLaserScan>& scans,std::vector<Eigen::Vector3d>& robot_poses);
        void generatemap(const float (*obs_position)[2],const int obs_nums);
        cv::Mat PublishMap(void);



};



#endif

