#ifndef SIMPLE_PATHPLANING_H_
#define SIMPLE_PATHPLANING_H_

#include <iostream>
#include <string>
#include <vector>
#include "math.h"
#include "dwa_module.h"
#include "../occupymap/occupymapsimple.h"

class SimpleGetSpeed{
    private:
        float dt_;
    public:
        SimpleGetSpeed(float dt):dt_(dt){};

        ~SimpleGetSpeed(){};

        bool crash_avoid(const float (*obs_position)[2],const int obs_nums);

        bool crash_avoid_pixel(OccupyMap &occum);

        //不考虑碰撞
        float * get_speed_simple(path_planning::DWA &dwaplanning,const robo_state x,const float peo[3],const float (*obs_position)[2],const int obs_nums,OccupyMap &occum);

        /*
            分段线性模型计算各方向速度取值(相当于Kp控制)
        */
        float Piecewise_linear_model(float error, float error_max, float error_min, float v_max, float v_min);


        float * get_speed_PVT(path_planning::DWA &dwaplanning,const robo_state x,const float peo[3],const float (*obs_position)[2],const int obs_nums);
        /*
            pvt运动规划
            输入：初始速度v0，在目标位置的速度vf，最大速度vmax，最小速度vmin，最大加速度amax，到目标点的距离distance，时间间隔DT默认0.01s,速度更新间隔time_gap假设为0.1
            输出：从当前位置（原点）、速度v0，到目标位置、速度vf之间的轨迹（速度、时间），返回time_gap=0.1 or total_time对应的速度值
            注意：输入distance是目标点的x或y坐标，故Vx，Vy需要分别计算；输入distance>0时对应pvt规划的速度均<0，此时最大速度应使用vmin
            返回值使用时间为0.1 or total_time对应的速度值作为发送的速度（使用total_time对应速度值vf时意味着不用0.1s就可以到达目标点了，离目标点的距离很小，所以发送vf【一般为0】）
            (这里考虑了加速度，所以发送的速度不是立即能达到的速度，而是下一次更新速度的时候应该达到的速度，这里设下一次更新速度离当前时刻的间隔为0.1)
        */
        float calculatePVT(double v0, double vf, double vmax, double vmin, double amax, double distance, double DT = 0.01, double time_gap = 0.1);
        
};



#endif

