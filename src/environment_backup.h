#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <time.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/wait.h>
#include <vector>
#include <string>
#include <chrono>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>


#include "render/render.h"
#include "DWA/dwa_module.h"
#include "DWA/simple_pathplaning.h"
// #include "DWA/trans_module.h"
#include "serialsend/serialsend.h"
#include "humandetect/humandetect.h"
#include "intelreader/intel_reader.h"
#include "conio/conio.h"
#include "readimu/readimu.h"
#include "occupymap/occupymapsimple.h"
#include "blindcompensate/blindcompensate.h"
#include "datasave/datasave.h"

#include <fstream>


float camera_height = 0.40;
float camera_height_back = 0.31;
trans_param forward2world = {0,0.165,0.022};//坐标系转换
trans_param back2world = {M_PI,0.093,-0.033};
std::vector<float> forwardcam{camera_height,58.4/180.0*M_PI,45.5/180.0*M_PI};
std::vector<float> backcam{camera_height_back,75.0/180.0*M_PI,62.0/180.0*M_PI};//相机的高度，水平视场角，垂直视场角

float obs_scan_height = 0.1;
float valid_obs_dist = 3;
float param_obs_dist = 0.05;

int KeyFrameNums = 6;//0.2,0.4,0.6,0.8,1.0,1.2
float KeyFrameDist = 0.2;

typedef float Position[2];

//保存图像等的数据结构
struct imageinfo{
    int frameid;
    double frametime;
    cv::Mat forwardrgb;
    cv::Mat forwarddepth;
    cv::Mat backrgb;
    cv::Mat backdepth;
    std::vector<cv::Point2f> joints;//forwardrgb中的关节点像素坐标
    std::vector<cv::Point3f> joints_coord;
    std::vector<double> maxval;

    imageinfo() = default;   // 添加默认构造函数

    imageinfo(int frameid_init,double frametime_init,cv::Mat forwardrgb_init,cv::Mat forwarddepth_init,cv::Mat backrgb_init,
    cv::Mat backdepth_init,std::vector<cv::Point2f> joints_init,std::vector<cv::Point3f> joints_coord_init,std::vector<double> maxval_init):
    frameid(frameid_init),
    frametime(frametime_init),
    forwardrgb(forwardrgb_init),
    forwarddepth(forwarddepth_init),
    backrgb(backrgb_init),
    backdepth(backdepth_init),
    joints(joints_init),
    joints_coord(joints_coord_init),
    maxval(maxval_init)
    {

    }
};


//保存地图等的数据结构
struct mapinfo{
    int frameid;
    double frametime;
    std::vector<float> peo;//人体位置
    std::vector<std::vector<float>> obs_position;//障碍物位置
    float yaw_origin;
    float yaw_current;
    
    // std::vector<cv::Point2f> forwardobs_pixels;//前向RGB中障碍物像素点位置
    // std::vector<cv::Point2f> backobs_pixels;
    // imu数据,栅格地图等

    mapinfo() = default;   // 添加默认构造函数

    mapinfo(int frameid_init,double frametime_init,float * peo_init,float (*obs_position_init)[2],float obs_nums_init,float yaw_origin_init,float yaw_current_init):
    frameid(frameid_init),
    frametime(frametime_init),
    peo(peo_init,peo_init+3),
    yaw_origin(yaw_origin_init),
    yaw_current(yaw_current_init),
    obs_position(obs_nums_init,std::vector<float>(2))
    {
        for(int i=0; i<obs_nums_init; i++){
            obs_position[i] = std::vector<float>{obs_position_init[i][0], obs_position_init[i][1]};
        }
    }
};

bool saveobs(std::vector<mapinfo> mapinfo_vector_save){
    std::fstream _fobs;
    _fobs.open(OBS_PATH, std::ios::out | std::ios::trunc);
    if(_fobs.is_open()){
        for(int i = 0; i < mapinfo_vector_save.size(); i++){
            _fobs << "关键帧frameid = " <<mapinfo_vector_save[i].frameid <<"\n";
            _fobs << "关键帧frametime = "<<mapinfo_vector_save[i].frametime <<"\n";
            _fobs << "小车原始朝向yaw_origin = "<<mapinfo_vector_save[i].yaw_origin <<"\n";
            _fobs << "小车当前朝向yaw_current = "<<mapinfo_vector_save[i].yaw_current <<"\n";
            for(int p = 0; p < mapinfo_vector_save[i].peo.size(); p++){
                _fobs << "人体姿态 = " <<mapinfo_vector_save[i].peo[p] <<"\n";
            }
            _fobs << std::fixed << std::setprecision(3);
            for (int j = 0; j < mapinfo_vector_save[i].obs_position.size(); j++)
            {
                _fobs << mapinfo_vector_save[i].obs_position[j][0] << "\t" << mapinfo_vector_save[i].obs_position[j][1] << "\n";
            }
        }
        std::cout<<std::dec<<"障碍物信息写入文件成功，总帧数 = "<<mapinfo_vector_save.size()<<std::endl;
        _fobs.close();
        return true;
    }
    else
    {
        std::cout<<"障碍物信息写入文件失败"<<std::endl;
        return false;
    }
};

void clearimgqueue(std::queue<imageinfo>& imageinfo_queue) {
	std::queue<imageinfo> empty;
	swap(empty, imageinfo_queue);
}

void clearmapqueue(std::queue<mapinfo>& mapinfo_queue) {
	std::queue<mapinfo> empty;
	swap(empty, mapinfo_queue);
}


inline double get_time_now(void){
    timeb t_now;
    ftime(&t_now);
    return t_now.time + t_now.millitm * 1e-3;
}

double get_time_gap(timeb t_time2,timeb t_time1){
    double time_gap = t_time2.time + t_time2.millitm * 1e-3 - (t_time1.time + t_time1.millitm * 1e-3);
    return time_gap;
}

Position * hybird_obs(Position * forward_obs_position,int forward_obs_nums,Position * back_obs_position,int back_obs_nums){
    int obs_nums = forward_obs_nums+back_obs_nums;
    float (*obs_position)[2] = new Position[obs_nums];
    for(int i = 0, j = 0; i < obs_nums; i ++){
        if(i < forward_obs_nums){
            obs_position[i][0] = forward_obs_position[i][0];
            obs_position[i][1] = forward_obs_position[i][1];
        }
        else{
            obs_position[i][0] = back_obs_position[j][0];
            obs_position[i][1] = back_obs_position[j][1];
            j++;
        }
    }
    return obs_position;
}

bool crash_avoid(const float (*obs_position)[2],const int obs_nums){
    for ( int j = 0; j < obs_nums; j++ ){//检测机器人后方1m正方形内的障碍物(0.55是略大于后向机器人盲区长度的值)
        if(-0.8<obs_position[j][0]&&obs_position[j][0]<0&&-0.25<obs_position[j][1]&&obs_position[j][1]<0.25){
            return true;
        }
    }    
    return false;
}

/*
    分段线性模型计算各方向速度取值(相当于Kp控制)
*/
float Piecewise_linear_model(float error, float error_max, float error_min, float v_max, float v_min){
    float cal_v;
    float k;
    if (error > error_max){
        cal_v = v_max;
    }
    else if (error > error_min && error <= error_max){
        k = v_max/(error_max - error_min);
        cal_v = k*(error - error_min);
    }
    else if(error > -error_min && error <= error_min){
        cal_v = 0;
    }
    else if(error > -error_max && error <= -error_min){
        k = v_max/(error_max - error_min);
        cal_v = k*(error + error_min);
    }
    else{
        cal_v = v_min;
    }
    return cal_v;
}

float * get_speed_simple_originobs(path_planning::DWA &dwaplanning,const robo_state x,mapinfo &minfo){
    int obs_nums = minfo.obs_position.size();
    float (*obs_position)[2] = new float[obs_nums][2];
    for(int i=0; i<obs_nums; i++){
        obs_position[i][0] = minfo.obs_position[i][0];obs_position[i][1] = minfo.obs_position[i][1];
    }
    float * peo = new float[3];
    for(int i=0; i<minfo.peo.size(); i++){
        peo[i] = minfo.peo[i];
    }


    float * speed = new float[3];
    float thetaerr = atan2(sin(minfo.yaw_origin - minfo.yaw_current), cos(minfo.yaw_origin - minfo.yaw_current));
    float cal_w = dwaplanning.Piecewise_linear_model(thetaerr);//使得运动过程中小车朝向不变始终与走廊平行
    if(peo[0]==INFINITY||peo[1]==INFINITY){
        speed[0] = 0;speed[1] = 0;speed[2] = cal_w;
        delete []peo;delete []obs_position;
        return speed;
    }

    float * robo_goal;
    robo_goal = dwaplanning.calc_robo_goal(peo,x,obs_position,obs_nums);
    if(robo_goal[0]>=-0.10 && robo_goal[0]<=0.10 &&
       robo_goal[1]>=-0.05 && robo_goal[1]<=0.05){
        //std:://cout <<"到达目标点"<<"\n";
        speed[0] = 0;speed[1] = 0;speed[2] = cal_w;
        delete []peo;delete []obs_position;delete []robo_goal;
        return speed;
    }

    speed[0] = Piecewise_linear_model(robo_goal[0], dwaplanning.get_cfg().x_max, dwaplanning.get_cfg().x_min, dwaplanning.get_cfg().max_speedx, dwaplanning.get_cfg().min_speedx);
    speed[1] = Piecewise_linear_model(robo_goal[1], dwaplanning.get_cfg().y_max, dwaplanning.get_cfg().y_min, dwaplanning.get_cfg().max_speedy, dwaplanning.get_cfg().min_speedy);
    speed[2] = cal_w;

    if(robo_goal[0]>=-0.10 && robo_goal[0]<=0.10){speed[0] = 0;}//分别调整三轴速度，并设定位置稳定区域
    if(robo_goal[1]>=-0.05 && robo_goal[1]<=0.05){speed[1] = 0;}

    //检查该速度是否碰撞
    bool crash = crash_avoid(obs_position,obs_nums);
    if(crash && speed[0]<0){
        speed[0] = 0;speed[1] = 0;speed[2] = cal_w;
    }  
    delete []peo;delete []obs_position;delete []robo_goal;
    return speed;
};

float * get_speed(path_planning::DWA &dwaplanning,const robo_state x,const float peo[3],const float (*obs_position)[2],const int obs_nums){
    float * speed = new float[3];
    if(peo[0]==INFINITY||peo[1]==INFINITY){
        //std:://cout <<"人体当前位置获取失败"<<"\n";
        speed[0] = 0;speed[1] = 0;speed[2] = 0;
        return speed;
    }
    float * robo_goal;
    robo_goal = dwaplanning.calc_robo_goal(peo,x,obs_position,obs_nums);

    //std:://cout<<"robo_goal = ["<<robo_goal[0]<<" , "<<robo_goal[1]<<" ]"<<"\n";
    /*
    陷入局部最优点时
    */
    //判断逻辑需要优化，否则容易造成机器人反复陷入局部最优点，震荡不收敛
    //(另一方面，由于人的朝向加入计算后容易给目标位置带来非线性震荡，使得机器人脱离局部最优点更容易了)
    // float dist_to_goal = sqrt(pow(x[0]-robo_goal[0],2)+pow(x[1]-robo_goal[1],2));
    // float abs_speed = sqrt(pow(x[3],2)+pow(x[4],2));
    // if((abs_speed < dwaplanning.get_cfg().robot_stuck_flag_cons)&&(dist_to_goal > 0.5)){
    //     //std:://cout<<"机器人陷入局部最优值,正在调整..."<<"\n";
    //     dwaplanning.change_robo_goal(M_PI/3);//放松角度约束，改变机器人目标位置，直到到达目标点附近,在setparameter重置角度约束
    //     //要是还是不行，考虑dwaplanning.change_robo_goal(x),x设置为一个范围内的量，遍历该范围使得小车脱离局部最优值
    //     delete []robo_goal;
    //     float * robo_goal = dwaplanning.calc_robo_goal(peo,x,obs_position,obs_nums);//参数变了，重新计算robo_goal
    //     //std:://cout<<"robo_goal_changed = ["<<robo_goal[0]<<" , "<<robo_goal[1]<<" ]"<<"\n";
    //     //不是必要立即重新计算robo_goal,因为如果真的是局部最优值的话，当前循环得到的速度仍然不会动
    // }
    
    float * dw;
    dw = dwaplanning.calc_dynamic_window(x);
    float theta;
    theta = dwaplanning.calc_camera_direction_error(peo,x);
    float cal_w;
    cal_w = dwaplanning.Piecewise_linear_model(theta);
    //因为这里的cal_w是我们期望达到的速度，没有用dw来约束
    //所以在采样时间内实际角速度并不能达到cal_w,而后续预测轨迹应当使用实际角速度,
    //而非期望速度cal_w,因此这里对cal_w做一个窗口限制，取最靠近cal_w的窗口极值
    //这个值理论上在采样时间内可以达到，故而可以用于后续的轨迹预测
    cal_w = dwaplanning.get_true_vz(cal_w, x);

    if(robo_goal[0]>=-0.10 && robo_goal[0]<=0.10 &&
       robo_goal[1]>=-0.05 && robo_goal[1]<=0.05){
        //std:://cout <<"到达目标点"<<"\n";
        speed[0] = 0;speed[1] = 0;speed[2] = cal_w;
        delete []robo_goal;delete []dw;
        return speed;
    }

    trajectory * tras;
    int tras_nums;
    int tras_state_nums;
    tras = dwaplanning.calc_trajectories(x,dw,cal_w,&tras_nums,&tras_state_nums);
    Eval_Trajectory eval;
    eval = dwaplanning.calc_eval_index(tras,tras_nums,tras_state_nums,obs_position,obs_nums,robo_goal);
    
    //检查是否到达目标点附近/机器人和人体距离,从而设定不同的参数
    dwaplanning.setparameter(x, robo_goal, peo);//目前都设置为end2goal_cost优先
    speed = dwaplanning.calc_optimal_speed(eval,tras,dw,cal_w);//需返回
    
    /*理论上已有障碍物补偿机制，结合DWA可实现无碰撞,但补偿机制在机器人长时间慢速运行时不准确，因此增加防碰撞机制*/
    bool crash = crash_avoid(obs_position,obs_nums);
    if(crash && speed[0]<0){
        //std:://cout <<"障碍物过近，已停止..."<<"\n";
        speed[0] = 0;speed[1] = 0;speed[2] = 0;
    }

    /*设定机制处理：机器人陷入速度为0无法动弹，且离目标点还有一定距离的情况
    类似Vx，W空间运动时，调整转向，但在这里行不通，因为影响相机朝向
    */
    // float dist_to_goal = sqrt(pow(x[0]-robo_goal[0],2)+pow(x[1]-robo_goal[1],2));
    // float abs_speed = sqrt(pow(speed[0],2)+pow(speed[1],2));
    // int obs_flag = 0;
    // if((abs_speed < dwaplanning.get_cfg().robot_stuck_flag_cons)&&(dist_to_goal > 0.1)){
    //     //std:://cout<<"机器人陷入局部最优值,正在调整..."<<"\n";
    //     speed[0] = -0.1;
    //     speed[1] = -0.1;
    // }

    delete []robo_goal;delete []dw;
    for(int i=0;i<tras_nums;i++){//释放内存
        delete [] (tras[i]);//多层指针，逐层释放
    }
    delete []tras;///
    delete []eval.end2goal_cost;delete []eval.id_map;delete []eval.obstacle_cost;delete []eval.start2goal_cost;

    return speed;
}



//根据上一个帧机器人的状态和当前机器人状态，修正人体朝向
void amend_human_state(float * previous_peo,float * peo){
    bool previous_invalid_flag = (previous_peo[0]==INFINITY&&previous_peo[1]==INFINITY&&previous_peo[2]==INFINITY);
    bool current_invalid_flag = (peo[0]==INFINITY&&peo[1]==INFINITY&&peo[2]==INFINITY);
    if(!previous_invalid_flag && previous_peo[2]!= INFINITY && !current_invalid_flag && peo[2]!= INFINITY){
        float theta = atan2(sin(peo[2]+M_PI),cos(peo[2]+M_PI));
        if(fabs(theta-previous_peo[2]) < fabs(peo[2]-previous_peo[2])){
            peo[2] = theta;
        }
        previous_peo[0] = peo[0];
        previous_peo[1] = peo[1];
        previous_peo[2] = peo[2];
    }
    else{//信息不足，无法修正
        previous_peo[0] = peo[0];
        previous_peo[1] = peo[1];
        previous_peo[2] = peo[2];
    }
}

//根据上一个帧人的状态和当前人的状态，计算人体朝向（人的速度方向）
float calc_human_direction(float * previous_peo,float * current_peo,double get_peo_gap){
    float human_direction;
    if(previous_peo[0] == INFINITY || current_peo[0] == INFINITY){
        human_direction = current_peo[2];
    }
    else{
        float vx = (current_peo[0] - previous_peo[0])/get_peo_gap;
        float vy = (current_peo[1] - previous_peo[1])/get_peo_gap;
        float V = sqrt(vx*vx+vy*vy);
        if(V < 0.1){//速度较小时，关节点向量计算出的人体朝向可用
            human_direction = current_peo[2];
        }
        else{//速度较大时，使用速度方向作为人体朝向
            human_direction = atan2(current_peo[1] - previous_peo[1], current_peo[0] - previous_peo[0]);//速度方向
        }
    }
    if(human_direction!=INFINITY){//无论人体前后，只计算速度所在直线朝着机器人这一侧的方向角
        if(human_direction>=-M_PI/2 && human_direction<0){
            human_direction = human_direction + M_PI;
        }
        if(human_direction>=0 && human_direction<=M_PI/2){
            human_direction = human_direction - M_PI;
        }
    }
    return human_direction;
}


void jointsrender(cv::Mat &img_show, std::vector<cv::Point2f> &joints, std::vector<double> &maxval,const double fps,const float* peo,const int cycle_times)
{   
    double thres = CONFIDENCE_THRES;
    cv::Point2d *joints_disp = new cv::Point2d[joints.size()]();
	for (int i = 0; i < joints.size(); i++)
	{
		joints_disp[i].x = double(joints[i].x) * img_show.cols;
		joints_disp[i].y = double(joints[i].y) * img_show.rows;
	}
	for (int i = 0; i < joints.size(); i++)
	{
		if (maxval[i] > thres)
		{
			cv::circle(img_show, joints_disp[i], 3, cv::Scalar(0, 255, 255), 1);
			cv::putText(img_show, std::to_string(i), joints_disp[i], cv::FONT_HERSHEY_PLAIN, 1.3, cv::Scalar(0, 0, 255), 1);
		}
	}
    cv::putText(img_show, "FPS: " + std::to_string(fps), cv::Point(0, 10), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0, 0, 255), 1);
    cv::putText(img_show, "x=" + std::to_string((int)(peo[0]*1000)) , cv::Point(0, 25), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 0, 255), 1);
    cv::putText(img_show, "y=" + std::to_string((int)(peo[1]*1000)) , cv::Point(0, 40), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 0, 255), 1);
    cv::putText(img_show, "z=" + std::to_string((int)(peo[2]/M_PI*180)) , cv::Point(0, 55), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 0, 255), 1);
    cv::putText(img_show, "cycle=" + std::to_string(cycle_times) , cv::Point(0, 70), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0, 0, 255), 1);
    delete[] joints_disp;

    // cv::imshow("jointsrender", img_show);
    // int key = cv::waitKey(1);
    // return key;

}


//均值滤波
float meanfilter(std::vector<float> filterwindow){
    float filtered;
    float sum = 0;
    int validnum = 0;
    for(int i = 0; i < filterwindow.size(); i++){
        if(filterwindow[i]!=INFINITY){
            if(filterwindow[i]>=-M_PI && filterwindow[i]<=-M_PI/2){//原先分布在[-pi,-pi/2)或(pi/2,pi]的不连续区间角度，转化为连续区间
                sum += filterwindow[i]+2*M_PI;
            }
            if(filterwindow[i]<=M_PI && filterwindow[i]>=M_PI/2){
                sum += filterwindow[i];
            }
            validnum++;
        }
    }
    if(validnum == 0){
        filtered = INFINITY;
    }
    else{
        filtered = sum/validnum;
        filtered = filtered>M_PI?(filtered-2*M_PI):filtered;
    }
    return filtered;
}




#endif
