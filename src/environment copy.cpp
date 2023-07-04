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

using namespace cv;
using namespace path_planning;
using namespace serial;


float camera_height = 0.40;
float camera_height_back = 0.31;
trans_param forward2world = {0,0.165,0.022};//坐标系转换
trans_param back2world = {M_PI,0.093,-0.033};
std::vector<float> forwardcam{camera_height,58.4/180.0*M_PI,45.5/180.0*M_PI};
std::vector<float> backcam{camera_height_back,75.0/180.0*M_PI,62.0/180.0*M_PI};//相机的高度，水平视场角，垂直视场角

float obs_scan_height = 0.1;
float valid_obs_dist = 3;
float param_obs_dist;

int KeyFrameNums = 6;//0.2,0.4,0.6,0.8,1.0,1.2
float KeyFrameDist = 0.2;


typedef float Position[2];

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
        if(-0.55<obs_position[j][0]&&obs_position[j][0]<0&&-0.25<obs_position[j][1]&&obs_position[j][1]<0.25){
            return true;
        }
    }    
    return false;
}

float * get_speed(DWA &dwaplanning,const robo_state x,const float peo[3],const float (*obs_position)[2],const int obs_nums){
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


int main(int argc, char *argv[]) {
    if (argc != 2){
        param_obs_dist = 0.05;
    } 
	else {
        param_obs_dist = atof(argv[1]);
    }

    // 建立模型
	HumanPoseEstimator model(initializeSampleParams());
	// 加载模型
	sample::gLogInfo << "Loading and running a GPU inference engine" << std::endl;
	if (!model.load()){
		//printf("load model failed");
	}
	samplesCommon::BufferManager buffers(model.mEngine);
	auto context = SampleUniquePtr<nvinfer1::IExecutionContext>(model.mEngine->createExecutionContext());
	if (!context){
		//printf("load model pointer failed");
	}
    // 获取关节点个数
	std::vector<std::string>::iterator iter = std::find(model.mParams.outputTensorNames.begin(),
		model.mParams.outputTensorNames.end(), "pred");
	int ind = iter - model.mParams.outputTensorNames.begin();
	int nJoints = model.mOutputDims[ind].d[1];
	std::vector<cv::Point2f> joints(nJoints);
	std::vector<double> maxval(nJoints);

    // /*启动intel相机*/
    IntelReader intel_reader(back2world);
	rs2::align align_to_color(RS2_STREAM_COLOR);//声明与谁对齐-与颜色流
	if (!intel_reader.init()){
		//std:://cout << "Intel相机未连接，请检查!" << std::endl;
	}
	//std:://cout<<"相机1已启动..."<<std::endl;

    /*启动astra相机*/
    AstraReader astra_reader(forward2world);
	if (!astra_reader.init()){
		//std:://cout << "Astra相机未连接，请检查!" << std::endl;
	}
    //std:://cout<<"相机2已启动..."<<std::endl;

    DWA dwaplanning;//初始化路径规划对象
    SimpleGetSpeed sgs(0.5);
    robo_state x = {0,0,0,0,0,0};//初始化机器人状态
    std::vector<float> current_speed{0,0,0};//当前真实速度

    serialsend ser("/dev/ttyUSB0",115200);//初始化串口对象
    imu::readimu imudata("/dev/ttyUSB1");//初始化imu对象
    float yaw_origin,yaw;

    int cycle_times = 1;
    int continous_missing_times = 0;
    float last_valid_peo[3] = {INFINITY,INFINITY,INFINITY};
    cv::Point3f invalid_point;invalid_point.x = 0;invalid_point.y = 0;invalid_point.z = 0;
    float previous_peo[3] = {INFINITY,INFINITY,INFINITY};
    float stride_thre = 30;//迈腿的判断阈值
    bool record_flag = false;//记录非迈腿状态下的人体朝向-标志位
    float previous_delta_direction = INFINITY;
    float sum_direction = 0;//记录非迈腿状态下的人体朝向-总和
    int num_direction = 0;//记录非迈腿状态下的人体朝向-数目
    float ave_direction = INFINITY;
    // previous_peo[2] = M_PI;//初始朝向设置为180°
    timeb t_start, t_end;
	timeb t_capture, t_get_peo;
    timeb t_depth,t_peo,t_obs,t_dwa,t_refresh_speed,t_debuginfo;
    timeb t_get_joints_coord_bydepth,t_save_joints_coord,t_get_human_state,t_get_human_direction,t_coordtrans;

    timeb t_save_depth_rgb_start,t_save_depth_rgb_end;
    timeb t_savejointpixel_start,t_savejointpixel_end;
    timeb t_render_forwardobs_start,t_render_forwardobs_end;
    timeb t_render_backobs_start,t_render_backobs_end;
    timeb t_compose_video_start,t_compose_video_end;
    timeb t_save_video_start,t_save_video_end;
    timeb t_cal_error_start,t_cal_error_end;
    timeb t_print_error_start,t_print_error_end;
    timeb t_del_pointer_start,t_del_pointer_end;

    timeb t_loc_updateforobs,t_beforesend;

    float stop[3] = {0,0,0};

    //均值滤波参数
    // int filtersize = 25;
    // std::vector<float> filterwindow(filtersize);
    // for(int i=0;i<filtersize;i++){filterwindow[i] = INFINITY;}

    // 步态估计器
    // std::vector<float> q{100, 100, 100, 100, 100, 100, 20, 20, //before
    //                      400, 400, 400, 400, 400, 400, 100, 100};
	// std::vector<float> r{500, 500, 500, 500, 500, 500, 100, 100};
	std::vector<float> q{100, 100, 100, 100, 100, 100, 20, 20,
                         100, 100, 100, 100, 100, 100, 20, 20};
	std::vector<float> r{1600, 1600, 1600, 1600, 1600, 1600, 320, 320};
	IntentionEstimator ie(1, q, r);

    //保存障碍物位置
    // std::fstream f_forwardobs,f_backobs,f_obs;
    // f_forwardobs.open(FORWARD_OBS_PATH, std::ios::out | std::ios::trunc);
    // f_backobs.open(BACK_OBS_PATH, std::ios::out | std::ios::trunc);
    // f_obs.open(OBS_PATH, std::ios::out | std::ios::trunc);
    std::vector<cv::Point2f> forwardobs_pixels(WIDTH);//保存每帧RGB中障碍物像素点位置
    std::vector<cv::Point2f> backobs_pixels(WIDTH);
    // std::vector<cv::Point2f> plane_pixels(WIDTH*HEIGHT);//保存每帧RGB中用于计算人体朝向的平面点位置

    // 创建数据保存对象
    DataSave ds;

    //障碍物盲区补偿对象
    BlindCompensate bc(KeyFrameNums, forwardcam, backcam, obs_scan_height);
    std::vector<float> lastkeyframe_roboloc{0,0,0};
    OccupyMap occum(bc);

    // //保存关节点像素坐标
    // std::fstream f_jointspixel;
    // f_jointspixel.open(JOINTS_PIXEL_PATH, std::ios::out | std::ios::trunc);
    // 保存机器人世界坐标系位置(初始点为原点)
    float robo_loc[3] = {0,0,0};
    double debuginfo_time = 0;

    // // 保存视频
	// cv::VideoWriter video_writer;
    // // video_writer.open("../testfile/resnet34.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 16, Size(WIDTH, HEIGHT));
    // video_writer.open("../testfile/resnet34.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 16, Size(2*WIDTH, HEIGHT));
    
    // cv::VideoWriter video_writer_back;
    // video_writer_back.open("../testfile/resnet34_back.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 16, Size(WIDTH, HEIGHT));

    while (true) {
        //std:://cout <<"********************"<<"\n";
        ftime(&t_start);
        //获取RGB图和点云
        // rs2::frameset frames = intel_reader.pipeline.wait_for_frames();
        // rs2::frame color_frame = frames.get_color_frame();
        // cv::Mat img_rgb(Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        cv::Mat img_rgb;
        astra_reader.read_rgb(img_rgb);

        cv::resize(img_rgb, img_rgb, cv::Size(256, 192), 0.0, 0.0, cv::INTER_CUBIC);//cv::Size(320, 240)
        // 推理获取关节点坐标和置信度,关节点保存于joints
        if (!model.infer(context, buffers, img_rgb, joints, maxval)){
            //printf("inference failed");
        }
        ftime(&t_capture);
        double cap_time = t_capture.time + t_capture.millitm * 1e-3 - (t_start.time + t_start.millitm * 1e-3);
        // cout << "捕获RGB+推理耗时cap_time =  " <<dec<< cap_time*1000 << "\n";


        //获取前向深度图
        cv::Mat depth_forward;
        astra_reader.read_depth(depth_forward);
        //获取后向深度图
        rs2::frameset frames = intel_reader.pipeline.wait_for_frames();
        frames = align_to_color.process(frames);//声明谁要对齐-采集获得的frames
        rs2::video_frame depth_frame = frames.get_depth_frame();
        cv::Mat depth_back(Size(depth_frame.get_width(), depth_frame.get_height()), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

        ftime(&t_depth);
        double depth_time = t_depth.time + t_depth.millitm * 1e-3 - (t_capture.time + t_capture.millitm * 1e-3);
        // cout << "获取深度图耗时depth_time =  " <<dec<< depth_time*1000 << "\n";


        //获取目标（人体）位置、朝向
        std::vector<cv::Point3f> joints_coord(nJoints);
        joints_coord = humandetect::get_joints_coord_bydepth(joints,maxval,depth_forward);

        ftime(&t_get_joints_coord_bydepth);
        // double get_joints_coord_bydepth_time = t_get_joints_coord_bydepth.time + t_get_joints_coord_bydepth.millitm * 1e-3 - (t_depth.time + t_depth.millitm * 1e-3);
        //cout << "获取关节坐标耗时get_joints_coord_bydepth_time =  " <<dec<< get_joints_coord_bydepth_time*1000 << "\n";

        // ds.addjoints(joints_coord, t_start.time + t_start.millitm * 1e-3);//保存关节点信息

        ftime(&t_save_joints_coord);
        // double save_joints_coord_time = t_save_joints_coord.time + t_save_joints_coord.millitm * 1e-3 - (t_get_joints_coord_bydepth.time + t_get_joints_coord_bydepth.millitm * 1e-3);
        //cout << "保存关节坐标耗时save_joints_coord_time =  " <<dec<< save_joints_coord_time*1000 << "\n";
        

        //float * current_peo = humandetect::get_human_state(joints_coord);
        float * current_peo = humandetect::get_human_state_knee(joints_coord);//优先使用膝关节计算人体位置
        if(current_peo[0] == INFINITY || current_peo[1] == INFINITY){//膝关节未获取到时，使用髋关节
            current_peo = humandetect::get_human_state(joints_coord);
        }  
        // std::cout <<"直接输出朝向 = "<<current_peo[2]/M_PI*180<<"\n";

        ftime(&t_get_human_state);
        // double get_human_state_time = t_get_human_state.time + t_get_human_state.millitm * 1e-3 - (t_save_joints_coord.time + t_save_joints_coord.millitm * 1e-3);
        //cout << "获取人体状态耗时get_human_state_time =  " <<dec<< get_human_state_time*1000 << "\n";

        // if(sqrt(x[3]*x[3]+x[4]*x[4]) > 0.1){//速度较大时关节点向量计算出的人体朝向不可用，转而使用人体速度获取人体朝向（效果不好）
        //     current_peo[2] = calc_human_direction(previous_peo,current_peo,get_peo_gap);
        // } 

        // filterwindow[(cycle_times-1)%filtersize] = current_peo[2];//均值滤波获取更可靠，光滑的人体朝向（效果不好）
        // current_peo[2] = meanfilter(filterwindow);

        // int plane_pixels_nums;//髋关节平面计算人体朝向（不好用）
        // current_peo[2] = humandetect::get_human_direction(joints,joints_coord,depth_forward,plane_pixels,&plane_pixels_nums);
        // float directionchange = fabs((current_peo[2]-previous_peo[2])/M_PI*180);

        /*imu获取走廊朝向作为人体朝向---更精准但不灵活（人只能在直线走廊行走才有效）*/
        // yaw = imudata.get_yaw();
        yaw = imudata.get_yaw_CH040();
        float temptheta = yaw - yaw_origin;//范围-90°~90°，超过时必已丢失目标
        std::cout <<"当前朝向yaw_current = "<<yaw/M_PI*180<<"\n";
        // std::cout <<"朝向偏差temptheta = "<<temptheta/M_PI*180<<"\n";
        current_peo[2] = M_PI - temptheta;//范围90°~270°
        current_peo[2] = atan2(sin(current_peo[2]), cos(current_peo[2]));//范围-180°~-90°并90°~180°

        
        // /*人体朝向的获取与补偿*/
        // ie.cal(joints_coord[0], joints_coord[2], joints_coord[3], joints_coord[5]); 
        // current_peo[2] = ie.get_human_direction();
        // // std::cout <<"滤波器输出朝向 = "<<current_peo[2]/M_PI*180<<"\n";
        // float delta_direction = fabs(atan2(-joints_coord[2].x + joints_coord[3].x, -joints_coord[2].z + joints_coord[3].z) - atan2(-joints_coord[1].x + joints_coord[4].x, -joints_coord[1].z + joints_coord[4].z));
        // delta_direction = delta_direction/M_PI*180.0 > 90 ? delta_direction/M_PI*180.0 - 180 : delta_direction/M_PI*180.0;
        // // std::cout <<"delta_direction = "<<delta_direction<<"\n";   
        // if(delta_direction > stride_thre){//迈腿时,设为前段时间平均朝向
        //     record_flag = false;
        //     sum_direction = 0;num_direction=0;
        //     current_peo[2] = ave_direction;
        // }
        // if(previous_delta_direction > stride_thre && delta_direction < stride_thre){
        //     record_flag = true;
        // }
        // if(record_flag){
        //     sum_direction += current_peo[2];num_direction++;
        //     ave_direction = sum_direction/num_direction;//num_direction该步至少为1
        // }
        // previous_delta_direction = delta_direction;
        // // std::cout <<"current_peo[2] = "<<current_peo[2]/M_PI*180<<"\n";
        // current_peo[2] = atan2(sin(current_peo[2] - M_PI/2), cos(current_peo[2] - M_PI/2));
        // if(current_peo[2]>=-M_PI/2 && current_peo[2]<0){
        //     current_peo[2] = current_peo[2] + M_PI;
        // }
        // if(current_peo[2]>=0 && current_peo[2]<=M_PI/2){
        //     current_peo[2] = current_peo[2] - M_PI;
        // }
        // current_peo[2] = atan2(sin(current_peo[2] + 7.5*M_PI/180), cos(current_peo[2] + 7.5*M_PI/180));//补偿稳态偏差
        // if(joints_coord[2]==invalid_point || joints_coord[3]==invalid_point){
        //     current_peo[2] = INFINITY;//无有效髋关节时,设为无有效朝向,而非滤波器得出的某一朝向
        // }

        /*人体偶然丢失后的补偿*/
        if(current_peo[0] != INFINITY && current_peo[1] != INFINITY){
            continous_missing_times=0;
        }
        if(current_peo[0] == INFINITY || current_peo[1] == INFINITY){
            continous_missing_times++;
            if(continous_missing_times==1){
                last_valid_peo[0] = previous_peo[0];last_valid_peo[1] = previous_peo[1];last_valid_peo[2] = previous_peo[2];
            }
        }
        if(continous_missing_times>=1 && continous_missing_times<=4){
            current_peo[0] = last_valid_peo[0];current_peo[1] = last_valid_peo[1];current_peo[2] = last_valid_peo[2];
            std::cout <<"人体偶然丢失补偿，continous_missing_times = "<<continous_missing_times<<"\n";
            //人体位置丢失帧数小于连续4帧时使用最后一帧的人体位置坐标代替人体位置
        }


        // if(delta_direction > stride_thre){//迈腿时,设为无有效朝向
        //     current_peo[2] = INFINITY;
        // } 
        // std::cout <<"current_peo[2] = *******************"<<current_peo[2]/M_PI*180<<"\n";

        ftime(&t_get_human_direction);
        // double get_human_direction_time = t_get_human_direction.time + t_get_human_direction.millitm * 1e-3 - (t_get_human_state.time + t_get_human_state.millitm * 1e-3);
        //cout << "获取人体朝向耗时get_human_direction_time =  " <<dec<< get_human_direction_time*1000 << "\n";


        // current_peo[2] = INFINITY;//不使用人体朝向

        timeb t_previous_peo = t_get_peo;
        ftime(&t_get_peo);
        double get_peo_gap = t_get_peo.time + t_get_peo.millitm * 1e-3 - (t_previous_peo.time + t_previous_peo.millitm * 1e-3);

        //std:://cout <<"previous_peo = [  "<<previous_peo[0]<< " , "<<previous_peo[1]<<" , "<<previous_peo[2]/M_PI*180<<" ° ]"<<"\n";
        //std:://cout <<"current_peo = [  "<<current_peo[0]<< " , "<<current_peo[1]<<" , "<<current_peo[2]/M_PI*180<<" ° ]"<<"\n";
        //std:://cout << "get_peo_gap =  " <<dec<< get_peo_gap*1000 << "\n";
        float * predict_peo =  dwaplanning.get_predict_peo(previous_peo,current_peo,get_peo_gap,x[3]);
        predict_peo[0] = current_peo[0];predict_peo[1] = current_peo[1];predict_peo[2] = current_peo[2];//不使用预测位置信息
        
        //std:://cout <<"predict_peo = [  "<<predict_peo[0]<< " , "<<predict_peo[1]<<" , "<<predict_peo[2]/M_PI*180<<" ° ]"<<"\n";
        previous_peo[0] = current_peo[0];previous_peo[1] = current_peo[1];previous_peo[2] = current_peo[2];

        //人体位置也要转换到同一坐标系
        if(predict_peo[0] != INFINITY){
            float peo_x = predict_peo[0];float peo_y = predict_peo[1];
            predict_peo[0] = peo_x*cos(forward2world.Theta) - peo_y*sin(forward2world.Theta) + forward2world.TranslationX;
            predict_peo[1] = peo_x*sin(forward2world.Theta) + peo_y*cos(forward2world.Theta) + forward2world.TranslationY;
        }
        // std::cout <<"predict_peo_after_trans = [  "<<predict_peo[0]<< " , "<<predict_peo[1]<<" , "<<predict_peo[2]/M_PI*180<<" ° ]"<<"\n";
        
        ftime(&t_coordtrans);
        // double coordtrans_time = t_coordtrans.time + t_coordtrans.millitm * 1e-3 - (t_get_human_direction.time + t_get_human_direction.millitm * 1e-3);
        //cout << "坐标转换耗时coordtrans_time =  " <<dec<< coordtrans_time*1000 << "\n";            
        
        ftime(&t_peo);
        // double peo_time = t_peo.time + t_peo.millitm * 1e-3 - (t_depth.time + t_depth.millitm * 1e-3);
        // cout << "获取人体位置、朝向耗时peo_time =  " <<dec<< peo_time*1000 << "\n";

        //获取障碍物位置
        Position * forward_obs_position;//前向
        int forward_obs_nums;
        forward_obs_position = astra_reader.depth2obs_astra(depth_forward, forwardobs_pixels, &forward_obs_nums, camera_height-obs_scan_height, param_obs_dist, valid_obs_dist);

        Position * back_obs_position_temp;//后向
        int back_obs_nums_temp;
        back_obs_position_temp = intel_reader.depth2obs_intel(depth_back, backobs_pixels, &back_obs_nums_temp, camera_height_back-obs_scan_height, param_obs_dist, valid_obs_dist);

        /*障碍物盲区补偿机制*/
        // ftime(&t_loc_updateforobs);
        // double loc_updateforobs_time = get_time_gap(t_loc_updateforobs,t_start);
        // //当前机器人位置获取更新（循环开始到现在的时间loc_updateforobs_time+上次机器人位置更新到循环结束的时间debuginfo_time==更新间隔）
        // robo_loc[2] = robo_loc[2] + current_speed[2]*(loc_updateforobs_time+debuginfo_time);
        // robo_loc[0] = robo_loc[0] + current_speed[0]*(loc_updateforobs_time+debuginfo_time)*cos(robo_loc[2]) + current_speed[1]*(loc_updateforobs_time+debuginfo_time)*cos(robo_loc[2]+M_PI/2);
        // robo_loc[1] = robo_loc[1] + current_speed[0]*(loc_updateforobs_time+debuginfo_time)*sin(robo_loc[2]) + current_speed[1]*(loc_updateforobs_time+debuginfo_time)*sin(robo_loc[2]+M_PI/2);
        // //当前人体位置（predict_peo不是一定真实位置不能用）
        // if(current_peo[0] != INFINITY){
        //     float peo_x = current_peo[0];float peo_y = current_peo[1];
        //     current_peo[0] = peo_x*cos(forward2world.Theta) - peo_y*sin(forward2world.Theta) + forward2world.TranslationX;
        //     current_peo[1] = peo_x*sin(forward2world.Theta) + peo_y*cos(forward2world.Theta) + forward2world.TranslationY;
        // }
        // keyframe currentframe(cycle_times,t_loc_updateforobs.time + t_loc_updateforobs.millitm * 1e-3,robo_loc,current_peo,back_obs_position_temp,back_obs_nums_temp);
        // if(bc.last_robo_loc(lastkeyframe_roboloc)){//上一关键帧存在
        //     float keyframedist = sqrt(pow(robo_loc[0]-lastkeyframe_roboloc[0],2)+pow(robo_loc[1]-lastkeyframe_roboloc[1],2));
        //     if(keyframedist > KeyFrameDist){//间隔第一次大于KeyFrameDist时，视为新的关键帧
        //         bc.addframe(currentframe);
        //     }
        // }
        // if(cycle_times == 1 || !bc.last_robo_loc(lastkeyframe_roboloc)){//第一个循环 or 上一关键帧不存在
        //     bc.addframe(currentframe);
        // }
        // // 调用obsfusion函数进行障碍物信息融合
        // std::vector<std::vector<float>> fuse_back_obs_position = bc.obsfusion(currentframe);
        // int back_obs_nums = fuse_back_obs_position.size();
        // delete []back_obs_position_temp;  float (*back_obs_position)[2] = new float[back_obs_nums][2];
        // for(int i=0; i<back_obs_nums; i++){
        //     back_obs_position[i][0] = fuse_back_obs_position[i][0];back_obs_position[i][1] = fuse_back_obs_position[i][1];
        // }

        int back_obs_nums = back_obs_nums_temp;
        float (*back_obs_position)[2] = new float[back_obs_nums][2];
        for(int i=0; i<back_obs_nums; i++){
            back_obs_position[i][0] = back_obs_position_temp[i][0];back_obs_position[i][1] = back_obs_position_temp[i][1];
        }
        delete []back_obs_position_temp;


        Position * obs_position;//总体
        int obs_nums = forward_obs_nums + back_obs_nums;
        obs_position = hybird_obs(forward_obs_position,forward_obs_nums,back_obs_position,back_obs_nums);
        ds.addobs(cycle_times,t_loc_updateforobs.time + t_loc_updateforobs.millitm * 1e-3,bc.get_frameids(),obs_position,obs_nums,predict_peo,yaw_origin,yaw);

        ////   ./environment 0.01
        occum.generatemap(obs_position,obs_nums);
        cv::Mat mapimage = occum.PublishMap();//可视化
        ds.addmap(mapimage);

        ftime(&t_obs);
        // double obs_time = t_obs.time + t_obs.millitm * 1e-3 - (t_peo.time + t_peo.millitm * 1e-3);
        //std:://cout << "障碍物数目 =  " << obs_nums << "\n";
        // cout << "获取障碍物位置耗时obs_time =  " <<dec<< obs_time*1000 << "\n";    


        //路径规划求速度
        // predict_peo[2] = INFINITY;
        float * speed;
        // speed = get_speed(dwaplanning, x, predict_peo, obs_position, obs_nums);
        // speed = sgs.get_speed_PVT(dwaplanning, x, predict_peo, obs_position, obs_nums);
        // speed = sgs.get_speed_simple(dwaplanning, x, predict_peo, obs_position, obs_nums);
        speed = sgs.get_speed_simple(dwaplanning, x, predict_peo, obs_position, obs_nums, occum);
        speed[2] = dwaplanning.Piecewise_linear_model(atan2(sin(-temptheta), cos(-temptheta)));//使得运动过程中小车朝向不变始终与走廊平行
        if(speed[0] == INFINITY || speed[1] == INFINITY || speed[2] == INFINITY){//simple方法无效时，使用dwa or 停止
            speed[0] = 0;speed[1] = 0;//speed[2] = 0;旋转速度不会导致碰撞
            // delete[] speed;
            // speed = get_speed(dwaplanning, x, predict_peo, obs_position, obs_nums);
            // speed[2] = dwaplanning.Piecewise_linear_model(atan2(sin(-temptheta), cos(-temptheta)));应用到DWA时简单加上该语句不行，因为DWA需要用角速度计算轨迹，更改角速度则轨迹全部需要重新计算
            cout << "simple无效，使用DAW\n";
        }
        ftime(&t_dwa);
        double dwa_time = t_dwa.time + t_dwa.millitm * 1e-3 - (t_obs.time + t_obs.millitm * 1e-3);
        // cout << "DWA耗时dwa_time =  " <<dec<< dwa_time*1000 << "\n";

        //发送速度
        if(cycle_times <= 10){//丢弃初始采集的10帧数据
            //std:://cout<<"该帧已丢弃..."<<"\n";
            speed[0] = 0;speed[1] = 0;speed[2] = 0;
            // yaw_origin = imudata.get_yaw();//作为走廊方向
            yaw_origin = imudata.get_yaw_CH040();//作为走廊方向
        }
        // std::cout <<"send_speed = [  "<<speed[0]<< " , "<<speed[1]<<" , "<<speed[2]/M_PI*180<<"°  ]"<<"\n";

        //发送下一条速度指令之前，再次更新机器人位置（用之前的速度）
        ftime(&t_beforesend);
        double update_delay_time = get_time_gap(t_beforesend,t_loc_updateforobs);//(上次机器人位置更新到当前的时间==更新间隔）
        robo_loc[2] = robo_loc[2] + current_speed[2]*update_delay_time;
        robo_loc[0] = robo_loc[0] + current_speed[0]*update_delay_time*cos(robo_loc[2]) + current_speed[1]*update_delay_time*cos(robo_loc[2]+M_PI/2);
        robo_loc[1] = robo_loc[1] + current_speed[0]*update_delay_time*sin(robo_loc[2]) + current_speed[1]*update_delay_time*sin(robo_loc[2]+M_PI/2);
        current_speed[0] = speed[0];current_speed[1] = speed[1];current_speed[2] = speed[2];//用完current_speed之后，进行更新（因为下一步就要发送新速度了）
        
        ser.send(speed);//速度发送

        //调试信息--一轮用时
        ftime(&t_end);
        double update_gap = t_end.time + t_end.millitm * 1e-3 - (t_start.time + t_start.millitm * 1e-3);
        // cout << "更新一轮耗时update_gap =  " <<dec<< update_gap*1000 << "\n";

        //&&&设置急停按钮，设置出错时发送停止信息&&&
        /* 将目标移除视野范围即可（遮挡摄像头） */

        //机器人状态更新
        x[3] = speed[0];
        x[4] = speed[1];
        x[5] = speed[2];
    
        ftime(&t_refresh_speed);
        // double refresh_speed_time = t_refresh_speed.time + t_refresh_speed.millitm * 1e-3 - (t_dwa.time + t_dwa.millitm * 1e-3);
        //cout << "发送更新速度耗时refresh_speed_time =  " <<dec<< refresh_speed_time*1000 << "\n";


        // //调试信息--可视化
        // cv::Mat img_depth(Size(WIDTH, HEIGHT), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
        // cv::resize(img_depth, img_depth, cv::Size(256, 192), 0.0, 0.0, cv::INTER_CUBIC);
        // cv::Mat img_show;
        // cv::threshold(img_depth, img_show, 1, 255, cv::THRESH_BINARY);
        // img_show.convertTo(img_show, CV_8UC1, 1, 0);
        // cv::cvtColor(img_show, img_show, cv::COLOR_GRAY2BGR);
        // cv::addWeighted(img_rgb, 0.7, img_show, 0.3, 0, img_show, CV_8UC3);
        // model.display(img_show, joints, maxval, 1 / update_gap, current_peo);

        // //保存深度图、RGB图/视频用于调试
        ftime(&t_save_depth_rgb_start);
        stringstream ss;
        ss << setw(10) << setfill('0') << cycle_times;
        if(cycle_times<10){
            String depth_file = "../testfile/depth/" + ss.str() + ".png";
            imwrite(depth_file, depth_forward);
        }
        // //保存RGB图（含关节点和障碍物位置）
        // cv::Mat forwardrgb;   // cv::Mat forwardrgb(Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        // astra_reader.read_rgb(forwardrgb);
        // jointsrender(forwardrgb, joints, maxval, 1.0 / update_gap, predict_peo, cycle_times);//渲染关节点
        // ds.addrgb_forward(forwardrgb);
        ftime(&t_save_depth_rgb_end);
        // double save_depth_rgb_time = t_save_depth_rgb_end.time + t_save_depth_rgb_end.millitm * 1e-3 - (t_save_depth_rgb_start.time + t_save_depth_rgb_start.millitm * 1e-3);
        // cout << "保存深度图及RGB图耗时save_depth_rgb_time =  " <<dec<< save_depth_rgb_time*1000 << "\n";

        ftime(&t_savejointpixel_start);
        // if (f_jointspixel.is_open())
        // {
        //     f_jointspixel << std::fixed << std::setprecision(6) << t_start.time + t_start.millitm * 1e-3;
        //     f_jointspixel << std::fixed << std::setprecision(3);
        //     for (int i = 0; i < joints.size(); i++){
        //         f_jointspixel << "\t" << joints[i].x << "\t" << joints[i].y;
        //     }
        //     f_jointspixel << "\n";
        // }
        ftime(&t_savejointpixel_end);
        // double savejointpixel_time = t_savejointpixel_end.time + t_savejointpixel_end.millitm * 1e-3 - (t_savejointpixel_start.time + t_savejointpixel_start.millitm * 1e-3);
        //cout << "保存关节点像素坐标耗时savejointpixel_time =  " <<dec<< savejointpixel_time*1000 << "\n"; 

        ftime(&t_render_forwardobs_start);
        // for(int i=0;i<forward_obs_nums;i++){//渲染障碍物
        //     cv::circle(forwardrgb, cv::Point(forwardobs_pixels[i].x, forwardobs_pixels[i].y), 3, cv::Scalar(255, 0, 255), 0.1);
        // }
        ftime(&t_render_forwardobs_end);
        // double render_forwardobs_time = t_render_forwardobs_end.time + t_render_forwardobs_end.millitm * 1e-3 - (t_render_forwardobs_start.time + t_render_forwardobs_start.millitm * 1e-3);
        // cout << "渲染前向障碍物耗时render_forwardobs_time =  " <<dec<< render_forwardobs_time*1000 << "\n"; 

        // for(int i=0;i<plane_pixels_nums;i++){//渲染用于计算人体朝向的平面点
        //     cv::circle(forwardrgb, cv::Point(plane_pixels[i].x, plane_pixels[i].y), 3, cv::Scalar(0, 0, 255), 0.1);
        // }
        // video_writer.write(forwardrgb);//保存视频

        ftime(&t_render_backobs_start);
        // rs2::frame color_frame = frames.get_color_frame();//cv::Mat backrgb;
        // cv::Mat backrgb(Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);//astra_reader.read_rgb(backrgb);
        // for(int i=0;i<back_obs_nums_temp;i++){//渲染障碍物
        //     cv::circle(backrgb, cv::Point(backobs_pixels[i].x, backobs_pixels[i].y), 3, cv::Scalar(255, 0, 255), 0.1);
        // }
        // cv::putText(backrgb, "cycle=" + std::to_string(cycle_times) , cv::Point(0, 10), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1);
        // cv::putText(backrgb, "back_obsnums_rgb=" + std::to_string(back_obs_nums_temp) , cv::Point(0, 25), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 255), 1);
        // cv::putText(backrgb, "back_obsnums_all=" + std::to_string(back_obs_nums) , cv::Point(0, 40), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0), 1);
        // ds.addrgb_back(backrgb);
        ftime(&t_render_backobs_end);
        // double render_backobs_time = t_render_backobs_end.time + t_render_backobs_end.millitm * 1e-3 - (t_render_backobs_start.time + t_render_backobs_start.millitm * 1e-3);
        // cout << "渲染后向障碍物耗时render_backobs_time =  " <<dec<< render_backobs_time*1000 << "\n";

        ftime(&t_compose_video_start);
        // cv::Mat ResultDisplay = Mat(HEIGHT, 2*WIDTH, CV_8UC3, Scalar::all(0));
        // Mat ROI_1 = ResultDisplay(Rect(0, 0, WIDTH, HEIGHT));Mat ROI_2 = ResultDisplay(Rect(WIDTH, 0, WIDTH, HEIGHT));
        // forwardrgb.copyTo(ROI_1);backrgb.copyTo(ROI_2);
        ftime(&t_compose_video_end);
        // double compose_video_time = t_compose_video_end.time + t_compose_video_end.millitm * 1e-3 - (t_compose_video_start.time + t_compose_video_start.millitm * 1e-3);
        //cout << "合成视频耗时compose_video_time =  " <<dec<< compose_video_time*1000 << "\n";

        ftime(&t_save_video_start);
        // video_writer.write(ResultDisplay);//保存视频
        ftime(&t_save_video_end);
        // double save_video_time = t_save_video_end.time + t_save_video_end.millitm * 1e-3 - (t_save_video_start.time + t_save_video_start.millitm * 1e-3);
        //cout << "保存视频耗时save_video_time =  " <<dec<< save_video_time*1000 << "\n";

        // //调试信息--保存每一帧图像对应相机朝向偏差、距离偏差、(机器人相对人体方向偏差)
        ftime(&t_cal_error_start);
        // float camera_direction_error = dwaplanning.calc_camera_direction_error(predict_peo,x);
        // float robot_direction_error = dwaplanning.calc_robot_direction_error(predict_peo,x);
        // float dist_to_peo = sqrt(pow(x[0]-predict_peo[0],2)+pow(x[1]-predict_peo[1],2));
        // if((predict_peo[0]==INFINITY&&predict_peo[1]==INFINITY&&predict_peo[2]==INFINITY)||(cycle_times <= 10)){
        //     camera_direction_error = INFINITY;
        //     robot_direction_error = INFINITY;
        //     dist_to_peo = INFINITY;
        // }
        ftime(&t_cal_error_end);
        // double cal_error_time = t_cal_error_end.time + t_cal_error_end.millitm * 1e-3 - (t_cal_error_start.time + t_cal_error_start.millitm * 1e-3);
        //cout << "计算偏差耗时cal_error_time =  " <<dec<< cal_error_time*1000 << "\n";
        
        ftime(&t_print_error_start);
        printf("cycle_times = %d\n",cycle_times);
        //std:://cout <<"camera_direction_error =  "<<camera_direction_error/M_PI*180<<" °"<<"\n";
        //std:://cout <<"robot_direction_error =  "<<robot_direction_error/M_PI*180<<" °"<<"\n";
        //std:://cout <<"dist_to_peo =  "<<dist_to_peo<<" m"<<"\n";
        ftime(&t_print_error_end);
        // double print_error_time = t_print_error_end.time + t_print_error_end.millitm * 1e-3 - (t_print_error_start.time + t_print_error_start.millitm * 1e-3);
        //cout << "打印偏差耗时print_error_time =  " <<dec<< print_error_time*1000 << "\n";

        
        ftime(&t_del_pointer_start);
        delete []predict_peo;
        delete []current_peo;
        delete []obs_position;//
        delete []forward_obs_position;
        delete []back_obs_position;//
        delete []speed;
        cycle_times++;
        if(kbhit()) {
            int key = getch();
            if(key == KEY_ESC) {//按ESC键退出
                break;
            }
        }
        ftime(&t_del_pointer_end);
        // double del_pointer_time = t_del_pointer_end.time + t_del_pointer_end.millitm * 1e-3 - (t_del_pointer_start.time + t_del_pointer_start.millitm * 1e-3);
        //cout << "删除野指针耗时del_pointer_time =  " <<dec<< del_pointer_time*1000 << "\n";


        ftime(&t_debuginfo);
        debuginfo_time = t_debuginfo.time + t_debuginfo.millitm * 1e-3 - (t_end.time + t_end.millitm * 1e-3);
        // cout << "输出调试信息耗时debuginfo_time =  " <<dec<< debuginfo_time*1000 << "\n";
        //std:://cout <<"  "<<"\n";//std:://cout <<"  "<<"\n";
    }
    ser.send(stop);
    // ds.savejoints();
    ds.saveobs();
    // ds.savergb_forward();
    // ds.savergb_back();
    ds.savemap();
    // video_writer.release();
    // video_writer_back.release();
    // f_jointspixel.close();
    std::cout<<"成功退出"<<std::endl;
    return 0;
}