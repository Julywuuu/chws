#include "dwa_module.h"

using namespace path_planning;
using namespace std;

/*
    获取private变量
*/
Config DWA::get_cfg(){
    return this->_cfg;
};


/*
    根据人体的位置朝向，计算机器人目标位置
    peo[3] = {X,Y,Theta}
    Theta取值范围为(-pi，pi]
*/
float * DWA::calc_robo_goal(const float peo[3],const robo_state x,const float (*obs_position)[2],const int obs_nums){
    float * robo_goal = new float[2];
    float alpha = this->_peo.optimal_relative_angel;
    float dist  = this->_peo.optimal_relative_dist;
    if (peo[2] == INFINITY){//表示人体朝向无法获取时，用于验证跟随算法阶段
        //人体-机器人连线的朝向作为人体朝向，后续就可以用else里的逻辑;但这样存在的问题就是,
        //人体朝向始终朝向机器人，那么小车每运动一下，robo_goal在小车坐标系中不变，小车永远无法到达robo_goal
        //【假设到达，那么此时optimal_relative_angel为0了，与设置冲突】
        // float theta = atan2(x[1] - peo[1], x[0] - peo[0]);
        // robo_goal[0] = peo[0] + dist*cos(alpha+theta);
        // robo_goal[1] = peo[1] + dist*sin(alpha+theta);
        // float temp = atan2(x[1] - peo[1], x[0] - peo[0]) - atan2(x[1] - peo[1], x[0] - peo[0]);
        // //std:://cout<<"current_relative_angel = "<<temp/M_PI*180<<"\n";
        
        //朝向无法获取,令机器人目标位置位于:人体位置为圆心,optimal_relative_dist为半径的圆上,距离机器人最近的那个点
        //即机器人和人体位置连线与圆的交点
        //cout<<"人体当前朝向获取失败"<<"\n";
        float k = sqrt(pow(dist,2)/(pow(x[0]-peo[0],2)+pow(x[1]-peo[1],2)));
        robo_goal[0] = k*(x[0]-peo[0]) + peo[0];
        robo_goal[1] = k*(x[1]-peo[1]) + peo[1];
    }
    else{//人体朝向可以获取时
        float delta = calc_robot_direction_error(peo,x);
        float speed = sqrt(x[3]*x[3]+x[4]*x[4]);
        // cout<<"delta = "<<delta/M_PI*180<<"\n";
        if(delta > 20 * M_PI / 180.0 || delta < -20 * M_PI / 180.0){//太侧面，人体朝向视为无效
            //cout<<"太侧面,人体朝向视为无效"<<"\n";
            float k = sqrt(pow(dist,2)/(pow(x[0]-peo[0],2)+pow(x[1]-peo[1],2)));
            robo_goal[0] = k*(x[0]-peo[0]) + peo[0];
            robo_goal[1] = k*(x[1]-peo[1]) + peo[1];            
        }
        // else if(speed < 0.05 && delta > -10 * M_PI / 180.0 && delta < 10 * M_PI / 180.0){//速度太小,且人体朝向偏差较小，人体朝向视为无效---->防止初始位置因人体迈步出现朝向测量错误
        //     //cout<<"初始运动,人体朝向视为无效"<<"\n";
        //     float k = sqrt(pow(dist,2)/(pow(x[0]-peo[0],2)+pow(x[1]-peo[1],2)));
        //     robo_goal[0] = k*(x[0]-peo[0]) + peo[0];
        //     robo_goal[1] = k*(x[1]-peo[1]) + peo[1];              
        // }
        else{
            float theta = peo[2];
            robo_goal[0] = peo[0] + dist*cos(alpha+theta);
            robo_goal[1] = peo[1] + dist*sin(alpha+theta);
        }
        // float theta = peo[2];
        // robo_goal[0] = peo[0] + dist*cos(alpha+theta);
        // robo_goal[1] = peo[1] + dist*sin(alpha+theta);//计算出该位置是理想位置(该位置可能有障碍物),需要结合障碍物信息来修正
        //理想位置被障碍物占据时,放松alpha的夹角约束
            //放松至[-xx,xx]
            //若还是没有合适位置,放松至[-pi,pi]
    }
    //std:://cout<<"optimal_relative_angel = "<<alpha/M_PI*180<<"\n";
    return robo_goal;
};


/*
    根据人体的位置朝向，计算机器人目标位置
    目标点确定的逻辑：先找出原先的目标点，也就是机器人人体连线和人体圆弧的交点；
    判断机器人当前位置到这个目标点的路径会不会碰到障碍物，如果不会，那么这个就是目标点，返回;
    如果会那么舍弃这个目标点，向目标点的两侧（圆弧的两侧）同时以固定间隔角度寻找，直到找到第一个
    合适的目标点（合适的目标点意思是机器人当前位置和这个合适目标点之间的路径和障碍物无碰撞）,返回;

补充（1）：最多向两侧各找二分之π
补充（2）：好像暂时并不需要这个（这个的等价替代可以是：全局路径规划+合适的跟随目标点确定算法）
【合适的跟随目标点标准，例如：位于合适距离，位于合适的角度，与障碍物有一定的距离，便于下一次的路径规划】
补充（3）：无碰撞要求太严格，在圆弧上找一个和障碍物距离大于碰撞距离的点即可；而后可结合全局路径规划使DWA规划出到达该目标点的速度

*/
float * DWA::calc_robo_goal_pro(const float peo[3],const robo_state x,const float (*obs_position)[2],const int obs_nums){
    float * robo_goal = new float[2];
    float alpha = this->_peo.optimal_relative_angel;
    float dist  = this->_peo.optimal_relative_dist;
    if (peo[2] == INFINITY){
        //cout<<"人体当前朝向获取失败"<<"\n";
        float k = sqrt(pow(dist,2)/(pow(x[0]-peo[0],2)+pow(x[1]-peo[1],2)));
        robo_goal[0] = k*(x[0]-peo[0]) + peo[0];
        robo_goal[1] = k*(x[1]-peo[1]) + peo[1];
    }
    else{//人体朝向可以获取时
        float theta = peo[2];
        robo_goal[0] = peo[0] + dist*cos(alpha+theta);
        robo_goal[1] = peo[1] + dist*sin(alpha+theta);//计算出该位置是理想位置(该位置可能有障碍物),需要结合障碍物信息来修正
        //理想位置被障碍物占据时,放松alpha的夹角约束
            //放松至[-xx,xx]
            //若还是没有合适位置,放松至[-pi,pi]
    }
    //std:://cout<<"optimal_relative_angel = "<<alpha/M_PI*180<<"\n";
    return robo_goal;
};


/*
    改变机器人目标位置（改变optimal_relative_angel）
*/
void DWA::change_robo_goal(const float optimal_relative_angel){
    this->_peo.optimal_relative_angel = optimal_relative_angel;
};


/*
    获取预测人体位置，预测时间间隔为更新一轮用时，约0.2s
*/
float * DWA::get_predict_peo(const float * previous_peo,const float * current_peo,const double get_peo_gap,const float previous_Vx){
    float * predict_peo = new float[3];
    float update_gap = 0.8;//预测时间,取小车调整到最优距离的时间
    if(previous_peo[0] == INFINITY || current_peo[0] == INFINITY){
        predict_peo[0] = current_peo[0];predict_peo[1] = current_peo[1];predict_peo[2] = current_peo[2];
    }
    else{
        float pre_d2peo = sqrt(pow(previous_peo[0],2)+pow(previous_peo[1],2));
        float cur_d2peo = sqrt(pow(current_peo[0],2)+pow(current_peo[1],2));
        if(cur_d2peo<this->_peo.optimal_relative_dist && cur_d2peo < pre_d2peo && previous_Vx < 0){
            float vx = (current_peo[0] - previous_peo[0])/get_peo_gap;
            float vy = (current_peo[1] - previous_peo[1])/get_peo_gap;
            predict_peo[0] = current_peo[0] + vx*update_gap;
            predict_peo[1] = current_peo[1];//y方向不进行预测
            predict_peo[2] = current_peo[2];
            // predict_peo[2] = (current_peo[2] + previous_peo[2])/2.0;//中值滤波
            //std:://cout<<"当前离人体距离小于最优距离and距离持续减小and上一发送速度Vx<0"<<"\n";
        }
        else{
            predict_peo[0] = current_peo[0];predict_peo[1] = current_peo[1];
            predict_peo[2] = current_peo[2];
            // predict_peo[2] = (current_peo[2] + previous_peo[2])/2.0;//中值滤波
        }
    }
    return predict_peo;
};

/*
    计算当前状态相机朝向，与相机-人体连线之间的夹角
    输入x为机器人当前状态
    #x = [x(m), y(m), yaw(rad), vx(m/s), vy(m/s), omega(rad/s)]
*/
float DWA::calc_camera_direction_error(const float peo[3],const robo_state x){
    float dx = peo[0] - x[0];
    float dy = peo[1] - x[1];
    float Cam_Peo_angle = atan2(dy, dx);//相机-人体连线，math.atan2是从-180出发逆时针单调递增至+180的函数
    float theta = Cam_Peo_angle - x[2];//x[2]相机朝向
    theta = atan2(sin(theta), cos(theta));//转化到-pi->pi之间
    return theta;
};


/*
    计算当前状态人体朝向，与人体-机器人连线之间的夹角
    输入x为机器人当前状态
    #x = [x(m), y(m), yaw(rad), vx(m/s), vy(m/s), omega(rad/s)]
*/
float DWA::calc_robot_direction_error(const float peo[3],const robo_state x){
    float dx = x[0] - peo[0];
    float dy = x[1] - peo[1];
    float Peo_Cam_angle = atan2(dy, dx);//人体-相机连线，math.atan2是从-180出发逆时针单调递增至+180的函数
    float theta = Peo_Cam_angle - peo[2];//peo[2]人体朝向
    theta = atan2(sin(theta), cos(theta));//转化到-pi->pi之间
    return theta;
};


/*
    分段线性模型计算角速度取值
*/
float DWA::Piecewise_linear_model(const float theta){
    float cal_w;
    float k;
    if (theta > this->_cfg.theta_max){
        cal_w = this->_cfg.max_yaw_rate;
    }
    else if (theta > this->_cfg.theta_min && theta <= this->_cfg.theta_max){
        k = this->_cfg.max_yaw_rate/(this->_cfg.theta_max - this->_cfg.theta_min);
        cal_w = k*(theta - this->_cfg.theta_min);
    }
    else if(theta > -this->_cfg.theta_min && theta <= this->_cfg.theta_min){
        cal_w = 0;
    }
    else if(theta > -this->_cfg.theta_max && theta <= -this->_cfg.theta_min){
        k = this->_cfg.max_yaw_rate/(this->_cfg.theta_max - this->_cfg.theta_min);
        cal_w = k*(theta + this->_cfg.theta_min);
    }
    else{
        cal_w = -this->_cfg.max_yaw_rate;
    }
    return cal_w;
};


/*
    对cal_w做一个窗口限制，取最靠近cal_w的窗口极值
    这个值理论上在采样时间内可以达到，故而可以用于后续的轨迹预测
*/
float DWA::get_true_vz(const float cal_w, const robo_state x){
    float true_vz;
    //理论上能够达到的最小速度
    float min = fmax(-this->_cfg.max_yaw_rate, x[5] - this->_cfg.max_delta_yaw_rate * this->_cfg.dt);
    //理论上能够达到的最大速度
    float max = fmin(this->_cfg.max_yaw_rate, x[5] + this->_cfg.max_delta_yaw_rate * this->_cfg.dt);
    if(cal_w > x[5]){
        if(cal_w > max){true_vz = max;}
        else{true_vz = cal_w;}
    }
    else{
        if(cal_w < min){true_vz = min;}
        else{true_vz = cal_w;}
    }
    return true_vz;
};


/*
    根据机器人当前速度得到采样的动态窗口
    返回[vx_min, vx_max, vy_min, vy_max]
*/
float * DWA::calc_dynamic_window(const robo_state x){
    float * dw = new float[4];
    dw[0] = fmax(this->_cfg.min_speedx, x[3] - this->_cfg.max_accelx * this->_cfg.dt);
    dw[1] = fmin(this->_cfg.max_speedx, x[3] + this->_cfg.max_accelx * this->_cfg.dt);
    // if(x[3]<this->_cfg.min_speedx){
    //     dw[0] = fmax(x[3], x[3] - this->_cfg.max_accelx * this->_cfg.dt);
    //     dw[1] = fmin(this->_cfg.max_speedx, x[3] + this->_cfg.max_accelx * this->_cfg.dt);
    // }
    dw[2] = fmax(this->_cfg.min_speedy, x[4] - this->_cfg.max_accely * this->_cfg.dt);
    dw[3] = fmin(this->_cfg.max_speedy, x[4] + this->_cfg.max_accely * this->_cfg.dt);
    //std:://cout<<"dwX = ["<<dw[0]<<","<<dw[1]<<"]  "<<"dwY = ["<<dw[2]<<","<<dw[3]<<"]"<<"\n";
    return dw;
};


/*
    运动学模型函数用于预测轨迹
    以当前状态x出发，用速度u运动dt时间后，机器人的状态
    #x = [x(m), y(m), yaw(rad), vx(m/s), vy(m/s), omega(rad/s)]
    #u = [vx,vy,w]
*/
float * DWA::motion(const robo_state x, const float u[3]){
    float dt = this->_cfg.dt;
    float * x_after_dt = new float[6];
    x_after_dt[2] = x[2] + u[2] * dt;
    x_after_dt[0] = x[0] + u[0]*dt*cos(x[2]) + u[1]*dt*cos(x[2]+M_PI/2);
    x_after_dt[1] = x[1] + u[0]*dt*sin(x[2]) + u[1]*dt*sin(x[2]+M_PI/2);

    x_after_dt[3] = u[0];
    x_after_dt[4] = u[1];
    x_after_dt[5] = u[2];
    return x_after_dt;
};


/*
    以当前状态x出发，用速度u运动predict_time时间，机器人所形成的轨迹
    #x = [x(m), y(m), yaw(rad), vx(m/s), vy(m/s), omega(rad/s)]
    #u = [vx,vy,w]
*/
trajectory DWA::predict_trajectory(const robo_state x, const float u[3], int * tras_state_nums){
    int cycle_times = (int)(this->_cfg.predict_time/this->_cfg.dt) + 1;
    *tras_state_nums = cycle_times;
    //bool tra_protect_flag = true;//开始时需要判断是否需要轨迹保护，一旦第一个不需要轨迹保护的点出现，则之后的点必定不再需要轨迹保护，将该变量设为false
    trajectory tra = new robo_state[cycle_times];
    for(int j = 0; j < 6; j++){
        tra[0][j] = x[j];
    }
    for(int i = 1; i < cycle_times; i++){
        float * temp = motion(tra[i-1],u);
        for(int j = 0;j<6;j++){
            tra[i][j] = temp[j];
        }
        // if(tra_protect_flag){
        //     float dist = sqrt(pow(tra[i-1][0]-tra[0][0],2)+pow(tra[i-1][1]-tra[0][1],2));//相对坐标系下始终有tra[0][0] = 0;tra[0][1]=0
        //     if(dist < this->_cfg.tra_protect_radius){
        //         for(int k = 0; k < 3; k++){
        //             tra[i-1][k] = INFINITY;//轨迹点tra[i-1]用于计算下一个轨迹点tra[i]的功能已经完成，所以在这里判断其是否位于轨迹保护范围
        //         }
        //     }
        //     else{
        //         tra_protect_flag = !tra_protect_flag;
        //     }
        // }
        delete []temp; 
    }
    
    // if(tra_protect_flag){//tra_protect_flag始终为true，那么除了末端外的所有轨迹点都在轨迹保护范围内，此时末端点未进行判断，这里补充判断
    //     float dist = sqrt(pow(tra[cycle_times-1][0]-tra[0][0],2)+pow(tra[cycle_times-1][1]-tra[0][1],2));
    //     if(dist < this->_cfg.tra_protect_radius){
    //         for(int k = 0; k < 3; k++){
    //             tra[cycle_times-1][k] = INFINITY;
    //         }
    //     }
    // }    
    // for(int k = 0; k < 3; k++){
    //     tra[0][k] = INFINITY;//设为无穷大，那么和障碍物的距离也为无穷大，即该点必定不会和障碍物碰撞，相当于碰撞检测时不考虑该点
    // }
    return tra;
};


/*
    根据当前状态，动态窗口以及角速度得到采样的各条轨迹
*/
trajectory * DWA::calc_trajectories(const robo_state x, const float * dw, const float cal_w, int * tras_nums, int * tras_state_nums){
    int cycle_times_vx = (int)((dw[1]-dw[0])/this->_cfg.v_resolutionx) + 1;
    int cycle_times_vy = (int)((dw[3]-dw[2])/this->_cfg.v_resolutiony) + 1;
    int nums_tra = cycle_times_vx*cycle_times_vy;
    *tras_nums = nums_tra;
    float tempvx = dw[0];
    float tempvy = dw[2];
    float tempv[3] = {tempvx,tempvy,cal_w};
    trajectory * tras = new trajectory[nums_tra];
    for(int i = 0; i < cycle_times_vx; i++){
        for(int j = 0; j < cycle_times_vy; j++){
            tempv[0] = tempvx;
            tempv[1] = tempvy;
            tras[i*cycle_times_vy+j] = this->predict_trajectory(x,tempv,tras_state_nums);
            tempvy += this->_cfg.v_resolutiony;
        }
        tempvy = dw[2];
        tempvx += this->_cfg.v_resolutionx;
    }
    return tras;
};


/*
    根据地图信息、机器人目标位置信息对各条轨迹进行评价
    tras_nums为轨迹数目
*/
Eval_Trajectory DWA::calc_eval_index(const trajectory * tras,const int tras_nums,const int tras_state_nums,const float (*obs_position)[2],const int obs_nums,const float * robo_goal){
    Eval_Trajectory eval;
    eval.tras_nums = 0;
    float obstacle_cost;
    eval.obstacle_cost = new float[tras_nums];
    eval.end2goal_cost = new float[tras_nums];
    eval.start2goal_cost = new float[tras_nums];
    eval.id_map = new int[tras_nums];
    for ( int i = 0; i < tras_nums; i++ ){
        obstacle_cost = this->calc_obstacle_cost(tras[i], tras_state_nums, obs_position, obs_nums);
        if(obstacle_cost == INFINITY)
            continue; 
        else{
            eval.obstacle_cost[eval.tras_nums] = obstacle_cost;
            eval.end2goal_cost[eval.tras_nums] = this->calc_end2goal_cost(tras[i], tras_state_nums, robo_goal);
            eval.start2goal_cost[eval.tras_nums] = this->calc_start2goal_cost(tras[i], tras_state_nums, robo_goal);
            eval.id_map[eval.tras_nums] = i;
            eval.tras_nums++;
        }

        // //cout<< tras[i] << "\n";
        // //cout << "********************" <<"\n";
    }
    return eval;
};


/*
    检查是否到达目标点附近,从而设定不同的参数
*/
void DWA::setparameter(const robo_state x, const float * robo_goal, const float peo[3]){
    /*进入刹车距离时进行参数切换*/
    // float Rx = x[3]*x[3]/(2*this->_cfg.max_accelx);
    // float Ry = x[4]*x[4]/(2*this->_cfg.max_accely);
    // float dx = abs(x[0]-robo_goal[0]);
    // float dy = abs(x[1]-robo_goal[1]);
    // //std:://cout<<"[Rx,Ry] = ["<<Rx<<","<<Ry<<"]"<<"\n";
    // //std:://cout<<"[dx,dy] = ["<<dx<<","<<dy<<"]"<<"\n";
    // if(dx < Rx || dy < Ry){
    //     this->_cfg.start2goal_cost_gain = 0;
    //     this->_cfg.end2goal_cost_gain = 1;
    //     //cout<<"到达目标点附近"<<"\n";
    // }
    // else{
    //     this->_cfg.start2goal_cost_gain = 1;
    //     this->_cfg.end2goal_cost_gain = 0;
    // }

    /*线性函数进行参数连续变化，斜率过大不好用*/
    // float dist_to_goal = sqrt(pow(x[0]-robo_goal[0],2)+pow(x[1]-robo_goal[1],2));
    // //dist=0.8,k=0.1;dist=0.3,k=1
    // float x1 = 0.8,y1 = 0.4;
    // float x2 = 0.2,y2 = 2.5;
    // float a = (y1-y2)/(x1-x2);
    // float b = y1-a*x1;
    // this->_cfg.start2goal_cost_gain = 1;
    // float k = a*dist_to_goal+b;
    // if(dist_to_goal < x2){
    //     k = y2;
    // }
    // if(dist_to_goal > x1){
    //     k = y1;
    // }
    // //std:://cout<<"dist_to_goal = "<< dist_to_goal <<"\n";
    // //std:://cout<<"k = "<< k <<"\n";
    // this->_cfg.end2goal_cost_gain = k*this->_cfg.start2goal_cost_gain;
    
    /*进入指定距离时进行参数切换*/
    float dist_to_goal = sqrt(pow(x[0]-robo_goal[0],2)+pow(x[1]-robo_goal[1],2));
    if(dist_to_goal < 0.35){//} || dist_to_goal < Rx){
        this->_cfg.start2goal_cost_gain = 0;
        this->_cfg.end2goal_cost_gain = 1;
        ////std:://cout<<"[Rx,dist_to_goal] = ["<<Rx<<","<<dist_to_goal<<"]"<<"\n";
        //cout<<"到达目标点附近"<<"\n";
    }
    if(dist_to_goal >= 0.35){
        this->_cfg.start2goal_cost_gain = 1;
        this->_cfg.end2goal_cost_gain = 0;
    }
    /*机器人和人体距离进入指定范围时进行参数切换*/
    // float dist_to_peo = sqrt(pow(x[0]-peo[0],2)+pow(x[1]-peo[1],2));
    // float max_dist = 2.7;//最远4.1m能识别到人体，留出一定余量
    // float less_dist = 2.1;
    // if(dist_to_peo > max_dist+0.5){//0.3是留出一定的裕度，防止机器人稳定在max_dist附近时吗目标距离产生波动
    //     //说明机器人和人体之间距离有扩大的趋势
    //     this->_peo.optimal_relative_dist=less_dist;
    // }
    // if(dist_to_peo < less_dist){
    //     this->_peo.optimal_relative_dist=max_dist;
    // }
    /*机器人和人体距离进入指定范围时进行参数切换*/
    float dist_to_peo = sqrt(pow(x[0]-peo[0],2)+pow(x[1]-peo[1],2));
    if(dist_to_peo < this->_peo.optimal_relative_dist){
        //说明机器人和人体之间距离有减少的趋势
        this->_cfg.start2goal_cost_gain = 1;
        this->_cfg.end2goal_cost_gain = 0;
    }
    /*速度进入指定范围时进行参数切换*/
    float speed = sqrt(x[3]*x[3]+x[4]*x[4]);
    if(speed < 0.25){//速度较小时，看更长的时间，这意味着更远的距离，从而可以预见远处的障碍物
        this->_cfg.dt = 0.4;
        this->_cfg.predict_time = 2.4;
    }
    else{
        this->_cfg.dt = 0.2;
        this->_cfg.predict_time = 1.2;
    }
    if(x[3] < 0 && speed > 0.3){
        this->_cfg.dt = 0.2;
        this->_cfg.predict_time = 1.0;        
    }
    //std:://cout <<"速度大小为:speed = "<<speed<<"   dt = "<<this->_cfg.dt<<"   predict_time = "<<this->_cfg.predict_time<<"\n";
    //std:://cout <<"参数为:start2goal_cost = "<<this->_cfg.start2goal_cost_gain<<"   end2goal_cost = "<<this->_cfg.end2goal_cost_gain<<"\n";


};


/*
    数组求和
*/
float DWA::calc_arraysum(const float * array, const int array_num){
    float sum = 0;
    for(int i = 0; i < array_num; i ++){
        sum += array[i];
    }
    return sum;
};


/*
    获取窗口内的刹车速度
*/
float DWA::get_brake_speed(float dw1,float dw2){
    float brake_speed;
    if(dw2 <= 0){
        brake_speed = dw2;
    }
    else if(dw2 > 0 && dw1 <= 0){
        brake_speed = 0;
    }
    else{
        brake_speed = dw1;
    }
    return brake_speed;
};


/*
    归一化评价各个轨迹后返回最优速度
*/           
float * DWA::calc_optimal_speed(const Eval_Trajectory eval,const trajectory * tras, const float * dw, const float cal_w){
    float * optimal_speed = new float[3];
    if(eval.tras_nums == 0){
        //cout<<"障碍物较密集、较近或速度较快,需减速"<<"\n";//即在dw窗口中选择最小速度
        optimal_speed[0] = this->get_brake_speed(dw[0],dw[1]);//dw[0]-->不一定是最小速度
        //optimal_speed[0] = -0.1;
        optimal_speed[1] = this->get_brake_speed(dw[2],dw[3]);//dw[2]-->不一定是最小速度
        optimal_speed[2] = cal_w;//cal_w
        return optimal_speed;
    }
    int optimal_idx;
    float total_costsum = INFINITY;
    float total_costsum_temp;
    float obstacle_costsum = this->calc_arraysum(eval.obstacle_cost,eval.tras_nums);
    float end2goal_costsum = this->calc_arraysum(eval.end2goal_cost,eval.tras_nums);
    float start2goal_costsum = this->calc_arraysum(eval.start2goal_cost,eval.tras_nums);
    for ( int i = 0; i < eval.tras_nums; i++ ){
        total_costsum_temp = this->_cfg.obstacle_cost_gain*eval.obstacle_cost[i]/obstacle_costsum 
        + this->_cfg.end2goal_cost_gain*eval.end2goal_cost[i]/end2goal_costsum 
        + this->_cfg.start2goal_cost_gain*eval.start2goal_cost[i]/start2goal_costsum;
        if(total_costsum_temp < total_costsum){
            total_costsum = total_costsum_temp;
            optimal_idx = eval.id_map[i];
        }
    }
    optimal_speed[0] = tras[optimal_idx][1][3];
    optimal_speed[1] = tras[optimal_idx][1][4];
    optimal_speed[2] = tras[optimal_idx][1][5];
    
    return optimal_speed;
};


/*
    障碍物评价函数
*/   
float DWA::calc_obstacle_cost(const trajectory tra,const int tras_state_nums,const float (*obs_position)[2],const int obs_nums){
    float obstacle_cost;
    float dist;
    bool tra_protect_flag = true;//开始时需要判断是否需要轨迹保护，一旦第一个不需要轨迹保护的点出现，则之后的点必定不再需要轨迹保护，将该变量设为false
    //使用圆形代表机器人进行碰撞检测，后续可优化
    /*没有障碍物时，单独处理*/
    if(obs_nums == 0){
        obstacle_cost = 1;
        return obstacle_cost;
    }
    /*起点离障碍物最小距离*/
    float start2obs_dist = INFINITY;
    for ( int j = 0; j < obs_nums; j++ ){
        float temp = sqrt(pow(tra[0][0]-obs_position[j][0],2)+pow(tra[0][1]-obs_position[j][1],2));
        if(temp < start2obs_dist){
            start2obs_dist = temp;
        }
    }
    /*起点位于碰撞距离内才需要轨迹保护机制*/
    if(start2obs_dist < this->_cfg.robot_radius){
        /*为快速性，先判断轨迹末端点*/
        float end2start_dist = sqrt(pow(tra[tras_state_nums-1][0]-tra[0][0],2)+pow(tra[tras_state_nums-1][1]-tra[0][1],2));
        float end2obs_dist = INFINITY;
        for ( int j = 0; j < obs_nums; j++ ){
            float temp = sqrt(pow(tra[tras_state_nums-1][0]-obs_position[j][0],2)+pow(tra[tras_state_nums-1][1]-obs_position[j][1],2));
            if(temp < end2obs_dist){
                end2obs_dist = temp;
            }
        }
        /*最后一个点位于保护范围或离最近障碍物的距离小于碰撞检测距离，则判定该轨迹不可行，否则转入从头判断*/
        if(end2start_dist < this->_cfg.tra_protect_radius || end2obs_dist < this->_cfg.robot_radius){
            return INFINITY;
        }
        /*从头判断先把保护flag判断直到false（判断轨迹点是否位于保护范围内），然后开始判断每个点到障碍物的最小距离，取出这些最小距离的最小距离
        若是最小距离的最小距离小于碰撞检测距离，则轨迹不可行，否则轨迹可行*/
        else{
            for ( int i = 0; i < tras_state_nums; i++ ){
                if(tra_protect_flag == true){
                    dist = sqrt(pow(tra[i][0]-tra[0][0],2)+pow(tra[i][1]-tra[0][1],2));//相对坐标系下始终有tra[0][0] = 0;tra[0][1]=0
                    if(dist >= this->_cfg.tra_protect_radius){
                        tra_protect_flag = false;
                    }
                }
                if(tra_protect_flag == false){
                    for ( int j = 0; j < obs_nums; j++ ){
                        dist = sqrt(pow(tra[i][0]-obs_position[j][0],2)+pow(tra[i][1]-obs_position[j][1],2));
                        if(dist < this->_cfg.robot_radius){
                            return INFINITY;//如果会碰到障碍物，就放弃这条轨迹
                        }
                    }
                }
            }
        }
    }
    /*不需要轨迹保护机制时的实现*/
    else{
        for ( int i = 0; i < tras_state_nums; i++ ){
            for ( int j = 0; j < obs_nums; j++ ){
                //从轨迹末端开始看,因为末端更有可能碰到障碍物
                dist = sqrt(pow(tra[tras_state_nums-1-i][0]-obs_position[j][0],2)+pow(tra[tras_state_nums-1-i][1]-obs_position[j][1],2));
                if(dist < this->_cfg.robot_radius){
                    return INFINITY;//如果会碰到障碍物，就放弃这条轨迹
                }
            }
        }
    }
    obstacle_cost = 1;//如果不会碰到障碍物则给以定值
    return obstacle_cost;
};


/*
    轨迹末端到目标点的距离，作为损失。显然，由于轨迹末端是两秒后的轨迹
    所以以此作为评价使其尽可能靠近目标点会使得小车越靠近目标点越需要减速使得轨迹末端靠近目标点
    这样收敛速度较慢
*/   
float DWA::calc_end2goal_cost(const trajectory tra,const int tras_state_nums,const float * robo_goal){
    float end2goal_cost;
    end2goal_cost = sqrt(pow(tra[tras_state_nums-1][0]-robo_goal[0],2)+pow(tra[tras_state_nums-1][1]-robo_goal[1],2));
    return end2goal_cost;
};


/*
    轨迹初始几个点到目标点的距离，作为损失。显然，由于初始几个点离小车很近，
    这样只看初始点会使得小车快速靠近目标点，但靠近目标点后可能由于速度较大而反复震荡
    因此快到达目标点，使用calc_end2goal_cost，便于减速收敛
*/   
float DWA::calc_start2goal_cost(const trajectory tra,const int tras_state_nums,const float * robo_goal){
    float start2goal_cost = 0;
    float dist;
    // for(int i = 0; i < 4; i ++){
    //     dist = sqrt(pow(tra[i][0]-robo_goal[0],2)+pow(tra[i][1]-robo_goal[1],2));
    //     start2goal_cost += (4-i)*dist;
    // }
    start2goal_cost = sqrt(pow(tra[1][0]-robo_goal[0],2)+pow(tra[1][1]-robo_goal[1],2));
    return start2goal_cost;
};