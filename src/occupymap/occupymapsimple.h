#ifndef OCCUPYMAPSIMPLE_H_
#define OCCUPYMAPSIMPLE_H_

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include "../blindcompensate/blindcompensate.h"

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
    public:
        MapParams mapParams;
        //地图指针
        unsigned char* pMap;

        unsigned char* currentmap;

        BlindCompensate &bc;

        OccupyMap(BlindCompensate &bc_init):bc(bc_init){
            SetMapParams();
        };

        ~OccupyMap(){    
            if(pMap != NULL)
                delete pMap;
            if(currentmap != NULL)
                delete currentmap;
        };

        //判断栅格是否在盲区
        bool gridindex_in_blind(GridIndex index);
        void SetMapParams(float x_range = 6.0,float y_range = 3.0,float resolution = 0.04);

        std::vector<float> ConvertGridIndex2World(GridIndex index);
        GridIndex ConvertWorld2GridIndex(double x,double y);
        int GridIndexToLinearIndex(GridIndex index);

        void generatemap(const float (*obs_position)[2],const int obs_nums);
        cv::Mat PublishMap(void);



};



#endif

