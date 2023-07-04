#ifndef DWA_MODULE_H_
#define DWA_MODULE_H_

#include <iostream>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include "math.h"
#include "stdio.h"
#include "float.h"


// typedef Eigen::Matrix<float, 1, 6> robo_state;
// typedef Eigen::Matrix<float, Eigen::Dynamic, 6> trajectory;

typedef float robo_state[6];
typedef robo_state * trajectory;


//trajectory.push_back(***);

struct Eval_Trajectory{
    float * obstacle_cost;
    float * end2goal_cost;
    float * start2goal_cost;
    int tras_nums;//有效轨迹数目
//Eval_Trajectory和tras之间不是一一对应，因为Eval_Trajectory时会抛弃一些轨迹
//因此用id_map存储与tras的对应关系
    int * id_map;
};


namespace path_planning {

    class People{
        public:
            float optimal_relative_dist = 1.8+0.165;//1.9;//最优的相对距离
            //最优的相对夹角（人体-机器人连线向量p2，与人体朝向向量p1的夹角）
            //p1逆时针旋转得p2时夹角为正,夹角范围(-pi,pi]
            float optimal_relative_angel = 0;// M_PI/6,optimal_relative_angel为正时，小车始终在人体朝向左侧
            
            //People(float dist = 3, float angel = 3);//给了构造函数就按照构造函数来初始化了
            //成员函数来暴露给外面的调用者使用
            //函数需要声明
            // int GetHour();
            // void SetHour(int nHour);
    };


    class Config{
        public:
            float max_speedx = 1.1;//0.5;//0.6;  max1.0//[m/s]
            float min_speedx = -1.1;  //-0.8  # [m/s] -0.25后面传感器安装后可以调快;
            float max_speedy = 0.4;//0.4           
            float min_speedy = -0.4;//-0.4         
            float max_yaw_rate = 32.0 * M_PI / 180.0; //60.0  [rad/s]

            float max_accelx = 1.4;  //0.6  max1.4  # [m/ss]
            float max_accely = 0.3;  //0.3  max0.8  # [m/ss]
            float max_delta_yaw_rate = 16 * M_PI / 180.0;   //30  103  [rad/ss] 
            
            float v_resolutionx = 0.04;  //  # [m/s]
            float v_resolutiony = 0.02;  //  # [m/s]
            float yaw_rate_resolution = 1.5 * M_PI / 180.0;  //  # [rad/s]

            float dt = 0.2;  //  # [s] Time tick for motion prediction
            float predict_time = 1.2;  //#2.0  # [s]

            float end2goal_cost_gain = 1.0;
            float start2goal_cost_gain = 1.0;
            float obstacle_cost_gain = 1.0;

            float robot_width = 0.4;
            float robot_length = 0.4;//矩形作精细碰撞检测
            //【当前设为0.55是因为障碍物补偿机制不健全，需要与crash_avoid函数配合实现无碰,0.55是比盲区稍大的值】
            //补偿机制不健全:小车可能撞墙时----长时间低速运行----定位误差大---关键帧丢失---丢失后仍然撞墙
            float robot_radius = 0.55;//粗碰撞检测-----0.35时参数关联blindcompensate中的参数
            float tra_protect_radius = 0.10;//轨迹保护半径
            float robot_stuck_flag_cons = 0.02;

            //相机朝向约束
            float theta_min = 1.0/180.0*M_PI;//3°的容许误差    asin(0.4/1.9);0.05//     y方向(横向)的允许偏差/optimal_relative_dist
            float theta_max = 30.0/180.0*M_PI;

            //位置约束（x，y方向容许偏差，用于pid计算速度）
            float x_min = 0.05;
            float x_max = 0.35;
            float y_min = 0.05;
            float y_max = 0.20;
            
    };


    class DWA{//输入人体的位置、朝向；地图信息(障碍物位置,未知区域,可行区域)；输出最优速度
        private:
            People _peo;
            Config _cfg;
        
        public:
            // void printclass();
            // People get_peo();
            Config get_cfg();

            //根据人体的位置朝向，计算机器人目标位置
            float * calc_robo_goal(const float peo[3],const robo_state x,const float (*obs_position)[2],const int obs_nums);
            float * calc_robo_goal_pro(const float peo[3],const robo_state x,const float (*obs_position)[2],const int obs_nums);

            //改变机器人目标位置
            void change_robo_goal(const float optimal_relative_angel);

            float * get_predict_peo(const float * previous_peo,const float * current_peo,const double get_peo_gap,const float previous_Vx);

            //根据人体的位置朝向和当前机器人位置朝向(0,0,0)计算相机朝向误差theta
            float calc_camera_direction_error(const float peo[3],const robo_state x);

            //计算当前状态人体朝向，与人体-机器人连线之间的夹角
            float calc_robot_direction_error(const float peo[3],const robo_state x);
            
            //根据分段线性模型得到角速度
            float Piecewise_linear_model(const float theta);

            //因为这里分段线性模型得到的cal_w是我们期望达到的速度，没有用dw来约束
            //所以在采样时间内实际角速度并不能达到cal_w,而后续预测轨迹应当使用实际角速度,
            //而非期望速度cal_w,因此这里对cal_w做一个窗口限制，取最靠近cal_w的窗口极值
            //这个值理论上在采样时间内可以达到，故而可以用于后续的轨迹预测
            float get_true_vz(const float cal_w, const robo_state x);

            //根据机器人当前速度得到采样的动态窗口
            float * calc_dynamic_window(const robo_state x);

            //运动学模型及轨迹计算
            float * motion(const robo_state x, const float u[3]);
            trajectory predict_trajectory(const robo_state x, const float u[3], int * tras_state_nums);

            //根据当前状态，动态窗口以及角速度得到采样的各条轨迹
            trajectory * calc_trajectories(const robo_state x, const float * dw, const float cal_w, int * tras_nums, int * tras_state_nums);

            //数组求和函数
            float calc_arraysum(const float * array, const int array_num);

            //根据地图信息、机器人目标位置信息对各条轨迹进行评价
            Eval_Trajectory calc_eval_index(const trajectory * tras,const int tras_nums,const int tras_state_nums,const float (*obs_position)[2],const int obs_nums,const float * robo_goal);

            //检查是否到达目标点附近,从而设定不同的参数
            void setparameter(const robo_state x, const float * robo_goal, const float peo[3]);

            //获取窗口内的刹车速度
            float get_brake_speed(float dw1,float dw2);

            //归一化评价各个轨迹后返回最优速度
            float * calc_optimal_speed(const Eval_Trajectory eval,const trajectory * tras, const float * dw, const float cal_w);

            //评价函数
            float calc_obstacle_cost(const trajectory tra,const int tras_state_nums,const float (*obs_position)[2],const int obs_nums);
            
            float calc_end2goal_cost(const trajectory tra,const int tras_state_nums,const float * robo_goal);

            float calc_start2goal_cost(const trajectory tra,const int tras_state_nums,const float * robo_goal);

            
    };



}


#endif

