#include "simple_pathplaning.h"
using namespace path_planning;

bool SimpleGetSpeed::crash_avoid(const float (*obs_position)[2],const int obs_nums){
    for ( int j = 0; j < obs_nums; j++ ){//检测机器人后方1m正方形内的障碍物(0.55是略大于后向机器人盲区长度的值)
        if(-0.8<obs_position[j][0]&&obs_position[j][0]<0&&-0.25<obs_position[j][1]&&obs_position[j][1]<0.25){
            return true;
        }
    }    
    return false;
}

//如果使用栅格地图进行DWA，那么还需要计算出一条轨迹经过哪些栅格，用于判断轨迹是否碰撞
bool SimpleGetSpeed::crash_avoid_pixel(OccupyMap &occum){
    double world_leftupx = -0.8;double world_leftupy = 0.25;
    double world_rightdownx = 0.0;double world_rightdowny = -0.25;
    GridIndex leftup = occum.ConvertWorld2GridIndex(world_leftupx,-world_leftupy);//碰撞判定左上角.符号是因为对齐像素坐标系
    GridIndex rightdown = occum.ConvertWorld2GridIndex(world_rightdownx,-world_rightdowny);//碰撞判定左上角
    // std::cout<< "leftup" << leftup.x << "," << leftup.y<<std::endl;
    // std::cout<< "rightdown" << rightdown.x << "," << rightdown.y<<std::endl;
    for (int y = leftup.y; y <= rightdown.y; y++) {
        for (int x = leftup.x; x <= rightdown.x; x++) {
            int index = y * occum.mapParams.width + x;
            if(occum.currentmap[index] == 255){//有障碍物
                // std::cout<< "有障碍物" <<std::endl;
                return true;
            }
        }
    }
    // std::cout<< "无障碍物" <<std::endl;
    return false;
}

//不考虑碰撞
float * SimpleGetSpeed::get_speed_simple(DWA &dwaplanning,const robo_state x,const float peo[3],const float (*obs_position)[2],const int obs_nums,OccupyMap &occum){
    float * speed = new float[3];
    if(peo[0]==INFINITY||peo[1]==INFINITY){
        speed[0] = 0;speed[1] = 0;speed[2] = 0;
        return speed;
    }
    float * robo_goal;
    robo_goal = dwaplanning.calc_robo_goal(peo,x,obs_position,obs_nums);
    
    float theta;
    theta = dwaplanning.calc_camera_direction_error(peo,x);
    float cal_w;
    cal_w = dwaplanning.Piecewise_linear_model(theta);//

    if(robo_goal[0]>=-0.10 && robo_goal[0]<=0.10 &&
       robo_goal[1]>=-0.05 && robo_goal[1]<=0.05){
        //std:://cout <<"到达目标点"<<"\n";
        speed[0] = 0;speed[1] = 0;speed[2] = cal_w;
        return speed;
    }

    speed[2] = cal_w;
    speed[0] = Piecewise_linear_model(robo_goal[0], dwaplanning.get_cfg().x_max, dwaplanning.get_cfg().x_min, dwaplanning.get_cfg().max_speedx, dwaplanning.get_cfg().min_speedx);
    speed[1] = Piecewise_linear_model(robo_goal[1], dwaplanning.get_cfg().y_max, dwaplanning.get_cfg().y_min, dwaplanning.get_cfg().max_speedy, dwaplanning.get_cfg().min_speedy);

    if(robo_goal[0]>=-0.10 && robo_goal[0]<=0.10){speed[0] = 0;}//分别调整三轴速度，并设定位置稳定区域
    if(robo_goal[1]>=-0.05 && robo_goal[1]<=0.05){speed[1] = 0;}

    //检查该速度是否碰撞
    // int tras_state_nums;
    // trajectory tra = dwaplanning.predict_trajectory(x,speed,&tras_state_nums);
    // float obstacle_cost = dwaplanning.calc_obstacle_cost(tra, tras_state_nums, obs_position, obs_nums);
    // if(obstacle_cost == INFINITY){//会碰撞
    //     speed[0] = INFINITY;speed[1] = INFINITY;speed[2] = INFINITY;//无效值
    // }
    // delete []robo_goal;delete [] tra;
    bool crash = crash_avoid_pixel(occum);
    // bool crash = crash_avoid(obs_position,obs_nums);
    if(crash && speed[0]<0){
        speed[0] = INFINITY;speed[1] = INFINITY;speed[2] = INFINITY;//无效值
    }  
    delete []robo_goal;
    return speed;
};

/*
    分段线性模型计算各方向速度取值(相当于Kp控制)
*/
float SimpleGetSpeed::Piecewise_linear_model(float error, float error_max, float error_min, float v_max, float v_min){
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
};


float * SimpleGetSpeed::get_speed_PVT(path_planning::DWA &dwaplanning,const robo_state x,const float peo[3],const float (*obs_position)[2],const int obs_nums){
    float * speed = new float[3];
    if(peo[0]==INFINITY||peo[1]==INFINITY){
        speed[0] = 0;speed[1] = 0;speed[2] = 0;
        return speed;
    }
    float * robo_goal;
    robo_goal = dwaplanning.calc_robo_goal(peo,x,obs_position,obs_nums);
    float theta;
    theta = dwaplanning.calc_camera_direction_error(peo,x);
    float cal_w;
    cal_w = dwaplanning.Piecewise_linear_model(theta);

    if(robo_goal[0]>=-0.10 && robo_goal[0]<=0.10 &&
       robo_goal[1]>=-0.05 && robo_goal[1]<=0.05){
        //std:://cout <<"到达目标点"<<"\n";
        speed[0] = 0;speed[1] = 0;speed[2] = cal_w;
        return speed;
    }

    speed[2] = cal_w;
    speed[0] = calculatePVT(x[3], 0, dwaplanning.get_cfg().max_speedx, dwaplanning.get_cfg().min_speedx, dwaplanning.get_cfg().max_accelx, robo_goal[0], 0.01, 0.05);
    speed[1] = calculatePVT(x[4], 0, dwaplanning.get_cfg().max_speedy, dwaplanning.get_cfg().min_speedy, dwaplanning.get_cfg().max_accely, robo_goal[1], 0.01, 0.05);

    //检查该速度是否碰撞
    // int tras_state_nums;
    // trajectory tra = dwaplanning.predict_trajectory(x,speed,&tras_state_nums);
    // float obstacle_cost = dwaplanning.calc_obstacle_cost(tra, tras_state_nums, obs_position, obs_nums);
    // if(obstacle_cost == INFINITY){//会碰撞
    //     speed[0] = INFINITY;speed[1] = INFINITY;speed[2] = INFINITY;//无效值
    // }
    // delete []robo_goal;delete [] tra;
    for ( int j = 0; j < obs_nums; j++ ){//检测机器人后方1m正方形内的障碍物(0.55是略大于后向机器人盲区长度的值)
        if(-0.8<obs_position[j][0]&&obs_position[j][0]<0&&-0.25<obs_position[j][1]&&obs_position[j][1]<0.25){
            if(speed[0] < 0){
                speed[0] = INFINITY;speed[1] = INFINITY;speed[2] = INFINITY;//无效值
            }
        }
    }    
    delete []robo_goal;
    return speed;
};

/*
    pvt运动规划
    输入：初始速度v0，在目标位置的速度vf，最大速度vmax，最小速度vmin，最大加速度amax，到目标点的距离distance，时间间隔DT默认0.01s,速度更新间隔time_gap假设为0.1
    输出：从当前位置（原点）、速度v0，到目标位置、速度vf之间的轨迹（速度、时间），返回time_gap=0.1 or total_time对应的速度值
    注意：输入distance是目标点的x或y坐标，故Vx，Vy需要分别计算；输入distance>0时对应pvt规划的速度均<0，此时最大速度应使用vmin
    返回值使用时间为0.1 or total_time对应的速度值作为发送的速度（使用total_time对应速度值vf时意味着不用0.1s就可以到达目标点了，离目标点的距离很小，所以发送vf【一般为0】）
    (这里考虑了加速度，所以发送的速度不是立即能达到的速度，而是下一次更新速度的时候应该达到的速度，这里设下一次更新速度离当前时刻的间隔为0.1)
*/
float SimpleGetSpeed::calculatePVT(double v0, double vf, double vmax, double vmin, double amax, double distance, double DT, double time_gap) {
    float speed = 0;
    if(distance > 0){
        double t1 = (vmax - v0) / amax;
        double t3 = (vf - vmax) / (-amax);
        double t2;

        double d1 = v0 * t1 + 0.5 * amax * t1 * t1;
        double d3 = vf * t3 + 0.5 * amax * t3 * t3;
        double d2 = distance - d1 - d3;

        float peakv;

        if(d2 > 0){
            peakv = vmax;
            t2 = d2 / vmax;
        }
        else{
            peakv = sqrt(amax*distance+0.5*v0*v0+0.5*vf*vf);
            t1 = (peakv - v0)/amax;
            t3 = (vf - peakv)/(-amax);
            t2 = 0;
        }
        if(time_gap <= t1){
            speed = v0 + amax * time_gap;
        }
        else if(time_gap > t1 && time_gap <= t1+t2){
            speed = peakv;
        }
        else if(time_gap > t1+t2 && time_gap <= t1+t2+t3){
            speed = peakv + (-amax) * (time_gap - t1 - t2);
        }
        else{
            speed = vf;
        }
    }

    if(distance < 0){
        double t1 = (vmin - v0) / (-amax);
        double t3 = (vf - vmin) / amax;
        double t2;

        double d1 = v0 * t1 + 0.5 * (-amax) * t1 * t1;
        double d3 = vf * t3 + 0.5 * (-amax) * t3 * t3;
        double d2 = distance - d1 - d3;

        float peakv;

        if(d2 < 0){
            peakv = vmin;
            t2 = d2 / vmin;
        }
        else{
            peakv = -sqrt(-amax*distance+0.5*v0*v0+0.5*vf*vf);
            t1 = (peakv - v0)/(-amax);
            t3 = (vf - peakv)/amax;
            t2 = 0;
        }
        if(time_gap <= t1){
            speed = v0 + (-amax) * time_gap;
        }
        else if(time_gap > t1 && time_gap <= t1+t2){
            speed = peakv;
        }
        else if(time_gap > t1+t2 && time_gap <= t1+t2+t3){
            speed = peakv + amax * (time_gap - t1 - t2);
        }
        else{
            speed = vf;
        }
    }
    return speed;

    // double total_time = t1 + t2 + t3;
    // // Create the PVT profile
    // std::vector<std::pair<double, double>> pvt_profile;

    // // Add the initial velocity and time
    // pvt_profile.push_back(std::make_pair(v0, 0.0));

    // // Add the acceleration phase
    // for (double t = 0.0; t < t1; t += DT) {
    //     double vel = v0 + amax * t;
    //     pvt_profile.push_back(std::make_pair(vel, t));
    // }

    // // Add the constant velocity phase
    // for (double t = t1; t < t1 + t2; t += DT) {
    //     pvt_profile.push_back(std::make_pair(vmax, t));
    // }

    // // Add the deceleration phase
    // for (double t = t1 + t2; t < total_time; t += DT) {
    //     double vel = vf + amax * (t - t1 - t2);//vf + amax * (total_time - t)
    //     pvt_profile.push_back(std::make_pair(vel, t));
    // }

    // // Add the final velocity and time
    // pvt_profile.push_back(std::make_pair(vf, total_time));
}