
#include "DWA/dwa_module.h"
#include "serialsend/serialsend.h"
#include <fstream>

using namespace path_planning;
using namespace serial;
using namespace std;

typedef float Position[2];


float * get_speed(DWA dwaplanning, robo_state x,float peo[3],float (*obs_position)[2],int obs_nums){
    float * robo_goal;
    robo_goal = dwaplanning.calc_robo_goal(peo,x,obs_position,obs_nums);
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

    trajectory * tras;
    int tras_nums;
    int tras_state_nums;
    tras = dwaplanning.calc_trajectories(x,dw,cal_w,&tras_nums,&tras_state_nums);
    Eval_Trajectory eval;
    eval = dwaplanning.calc_eval_index(tras,tras_nums,tras_state_nums,obs_position,obs_nums,robo_goal);
    
    //检查是否到达目标点附近,从而设定不同的参数
    dwaplanning.setparameter(x, robo_goal);
    float * speed = new float[3];
    speed = dwaplanning.calc_optimal_speed(eval,tras,dw,cal_w);//需返回

    delete []robo_goal;delete []dw;
    for(int i=0;i<tras_nums;i++){//释放内存
        delete [] (tras[i]);
    }
    delete []tras;///
    delete []eval.end2goal_cost;delete []eval.id_map;delete []eval.obstacle_cost;delete []eval.start2goal_cost;

    return speed;
}

void printobs(float (*obs_position)[2],int obs_nums){
    for(int i = 0; i < obs_nums; i ++){
        cout<< "("<<obs_position[i][0] <<","<<obs_position[i][1] <<")"<<endl;
    }
}

bool write_error(ofstream &outfile, int cycle_times, float camera_direction_error, float robot_direction_error, float dist_to_peo){
    if (!outfile)
    {
        cout << "打开文件失败" << endl;
        return false;
    }
    //向文件中写入数据
    outfile << cycle_times <<"  "<< 
    camera_direction_error <<"  "<<
    robot_direction_error  <<"  "<<
    dist_to_peo<< endl;
    return true;
}

float calc_robot_direction_error(const float peo_theta,const float Peo_Cam_angle){
    float Peo_Cam_angle = Peo_Cam_angle;//相机-人体连线，math.atan2是从-180出发逆时针单调递增至+180的函数
    float theta = Peo_Cam_angle - peo_theta;//peo_theta人体朝向
    theta = atan2(sin(theta), cos(theta));//转化到-pi->pi之间
    return theta;
};

int main(int argc, char **argv) {


    ofstream outfile;    //定义输出流对象
    outfile.open("../src/sensors/errordata/errordata.txt");    //打开文件
    write_error(outfile,1,2,3,4);
    write_error(outfile,2,INFINITY,INFINITY,INFINITY);
    outfile.close();    //关闭文件

    // float speed[3] = {0.1,0,0.0};
    // float speed_stop[3] = {0,0,0};
    // serialsend ser("/dev/ttyUSB0",115200);
    // while(1){
    //     ser.send(speed);
    //     sleep(10);
    //     break;
    // }
    // ser.send(speed_stop);


    return 0;
}

// int main(int argc, char **argv) {
//     std::cout << "starting enviroment" << std::endl;

//     DWA dwaplanning;//初始化路径规划对象
//     robo_state x = {0,0,0,0,0,0};//初始化机器人状态

//     serialsend ser("/dev/ttyTHS1",115200);//初始化串口对象

//     int cycle_times = 1;
//     while (1) {
//         cout << "***cycle_times = " << cycle_times<<endl;
//         float obs_position[][2] = {
//             {0,1},
//             {1,1}
//         };
//         int obs_nums = sizeof(obs_position)/sizeof(obs_position[0]);
//         //cout<< obs_nums <<endl;
//         //printobs(obs_position,obs_nums);
        

//         float peo[3] = {3,0,3.1415};//需要加入获取人体位置模块
//         float * speed;
//         speed = get_speed(dwaplanning, x, peo, obs_position, obs_nums);
//         //发送速度
//         for(int i=0;i<3;i++){cout << speed[i] <<"  "<<endl;}
//         ser.send(speed);
//         //设置急停按钮，设置出错时发送停止信息
//         //机器人状态更新,每0.1s更新一次速度
//         sleep(0.1);
//         x[3] = speed[0];
//         x[4] = speed[1];
//         x[5] = speed[2];

//         delete []speed;///

//         cycle_times++;

//     }
// }