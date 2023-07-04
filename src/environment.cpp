#include "environment.h"

using namespace cv;
using namespace path_planning;
using namespace serial;

// 标志位，用于告知后台线程主线程已经退出
bool is_exit = false;
// 数据队列
std::queue<imageinfo> imageinfo_queue_save;//用于主线程产生数据，线程3保存数据
std::queue<imageinfo> imageinfo_queue_use;//用于主线程产生数据，线程1使用数据
std::vector<mapinfo> mapinfo_vector_save;//用于线程1产生数据，线程1结束前统一保存数据(或考虑线程4并行保存数据--那么这里用queue)
std::queue<mapinfo> mapinfo_vector_use;//用于线程1产生数据，线程2使用数据

// 互斥锁和条件变量，用于同步线程间的操作，对应四个数据队列
std::mutex image_mutex_save,image_mutex_use,map_mutex_save,map_mutex_use;

//image_cv---主线程通知线程1获取imageinfo_queue_use中的数据、主线程通知线程1可以退出;
//map_cv---线程1通知线程2从mapinfo_vector_use末尾获取数据、主线程通知线程2可以退出；
//save_cv---主线程通知线程3保存imageinfo_queue中的数据、主线程通知线程3可以退出；
std::condition_variable image_cv,map_cv,save_cv;

AstraReader astra_reader(forward2world);        //humandetect.cpp中extern了该对象
AstraReader astra_reader_back(forward2world);


// 线程1
void generate_map_thread() {

    imu::readimu imudata("/dev/ttyUSB0");//初始化imu对象
    float yaw_origin, yaw;
    int continous_missing_times = 0;
    float last_valid_peo[3] = {INFINITY,INFINITY,INFINITY};
    float previous_peo[3] = {INFINITY,INFINITY,INFINITY};

    std::vector<cv::Point2f> forwardobs_pixels(WIDTH);//保存每帧RGB中障碍物像素点位置
    std::vector<cv::Point2f> backobs_pixels(WIDTH);
    int forward_obs_nums, back_obs_nums, obs_nums;

    // //障碍物盲区补偿对象
    // BlindCompensate bc(KeyFrameNums, forwardcam, backcam, obs_scan_height);
    // OccupyMap occum(bc);
    
    while (true) {
        // 获取队列中的数据
        std::unique_lock<std::mutex> image_lock_use(image_mutex_use);
        image_cv.wait(image_lock_use, [](){ return !imageinfo_queue_use.empty() || is_exit; });
        if (is_exit && imageinfo_queue_use.empty()) {
            break;// 如果主线程已经退出并且队列为空，则退出循环
        }

        imageinfo imginfo = imageinfo_queue_use.back();//末尾数据是最新数据
        clearimgqueue(imageinfo_queue_use);//清空队列
        // std::cout << "线程1--img队列长度：" << imageinfo_queue_use.size()<< std::endl;
        image_lock_use.unlock();        

        //**************生成地图等的操作
        /*获取转换到机器人坐标系的人体位置*/
        float * current_peo = humandetect::get_human_state_knee(imginfo.joints_coord);//优先使用膝关节计算人体位置
        if(current_peo[0] == INFINITY || current_peo[1] == INFINITY){//膝关节未获取到时，使用髋关节
            current_peo = humandetect::get_human_state(imginfo.joints_coord);
        }
        //imu获取走廊朝向作为人体朝向---更精准但不灵活（人只能在直线走廊行走才有效）
        if(imginfo.frameid < 10){yaw_origin = imudata.get_yaw_CH040();}
        yaw = imudata.get_yaw_CH040();
        current_peo[2] = atan2(sin(M_PI-(yaw - yaw_origin)), cos(M_PI-(yaw - yaw_origin)));//范围-180°~-90°并90°~180°
        //人体偶然丢失后的补偿
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
            // std::cout <<"线程1--人体偶然丢失补偿，continous_missing_times = "<<continous_missing_times<<std::endl;
        }
        previous_peo[0] = current_peo[0];previous_peo[1] = current_peo[1];previous_peo[2] = current_peo[2];
        //人体位置也要转换到同一坐标系
        if(current_peo[0] != INFINITY){
            float peo_x = current_peo[0];float peo_y = current_peo[1];
            current_peo[0] = peo_x*cos(forward2world.Theta) - peo_y*sin(forward2world.Theta) + forward2world.TranslationX;
            current_peo[1] = peo_x*sin(forward2world.Theta) + peo_y*cos(forward2world.Theta) + forward2world.TranslationY;
        }

        /*获取障碍物位置*/
        Position * forward_obs_position = astra_reader.depth2obs_astra(imginfo.forwarddepth, forwardobs_pixels, &forward_obs_nums, camera_height-obs_scan_height, param_obs_dist, valid_obs_dist);
        Position * back_obs_position = astra_reader_back.depth2obs_astra(imginfo.backdepth, backobs_pixels, &back_obs_nums, camera_height_back-obs_scan_height, param_obs_dist, valid_obs_dist);
        //Position * back_obs_position = intel_reader.depth2obs_intel(imginfo.backdepth, backobs_pixels, &back_obs_nums, camera_height_back-obs_scan_height, param_obs_dist, valid_obs_dist);
        obs_nums = forward_obs_nums + back_obs_nums;
        Position * obs_position = hybird_obs(forward_obs_position,forward_obs_nums,back_obs_position,back_obs_nums);
        // occum.generatemap(obs_position,obs_nums);
        // cv::Mat mapimage = occum.PublishMap();//可视化    

        /*保存地图信息*/
        mapinfo minfo(imginfo.frameid,get_time_now(),current_peo,obs_position,obs_nums,yaw_origin,yaw);
        std::unique_lock<std::mutex> map_lock_use(map_mutex_use);//保存地图数据用于线程2使用
        mapinfo_vector_use.push(minfo);
        // std::cout << "线程1--use队列长度：" << mapinfo_vector_use.size()<< std::endl;//只会是1才对
        map_lock_use.unlock();
        map_cv.notify_one();//通知线程2从mapinfo_vector_use末尾获取数据

        //由于没有多线程并行操作mapinfo_vector_save，所以这里应该不用锁也行，当然用了也不会有线程竞争，可以立即获得锁
        std::unique_lock<std::mutex> map_lock_save(map_mutex_save);//保存地图数据用于存入硬盘
        mapinfo_vector_save.push_back(minfo);
        // std::cout << "线程1--save向量长度：" << mapinfo_vector_save.size()<< std::endl;
        map_lock_save.unlock();

        delete []current_peo;
        delete []obs_position;
        delete []forward_obs_position;
        delete []back_obs_position;
    }

    //****************保存mapinfo_vector_save的操作
    saveobs(mapinfo_vector_save);
    std::cout<<"线程1--thread1_generatemap exited"<<std::endl;
}

// 线程2
void path_planing_thread() {

    DWA dwaplanning;//初始化路径规划对象
    robo_state x = {0,0,0,0,0,0};//初始化机器人状态
    float stop[3] = {0,0,0};
    serialsend ser_front("/dev/ttyTHS0",115200);//初始化串口对象 与 stm32通信的接口
    serialsend ser_back("/dev/ttyTCU0",115200);//初始化串口对象 与 stm32通信的接口
    while (true) {
        // 获取mapinfo_vector中的数据
        std::unique_lock<std::mutex> map_lock_use(map_mutex_use);
        map_cv.wait(map_lock_use, [](){ return !mapinfo_vector_use.empty() || is_exit; });
        if (is_exit && mapinfo_vector_use.empty()) {
            break;// 如果主线程已经退出并且队列为空，则退出循环
        }
        mapinfo minfo = mapinfo_vector_use.back();//末尾数据是最新数据
        clearmapqueue(mapinfo_vector_use);//清空队列
        map_lock_use.unlock();     

        //****************路径规划等的操作
        //路径规划
        // std::cout <<"current_peo = [  "<<minfo.peo[0]<< " , "<<minfo.peo[1]<<" , "<<minfo.peo[2]/M_PI*180<<" ° ]"<<std::endl;
        float * speed = get_speed_simple_originobs(dwaplanning, x, minfo);
        //发送速度
        if(minfo.frameid <= 10){//丢弃初始采集的10帧数据
            speed[0] = 0;speed[1] = 0;speed[2] = 0;
        }
        std::cout <<"线程2--send_speed = [  "<<speed[0]<< " , "<<speed[1]<<" , "<<speed[2]/M_PI*180<<"°  ]"<<std::endl;

        ser_front.send(speed);
        ser_back.send(speed);

        //机器人状态更新
        x[3] = speed[0];x[4] = speed[1];x[5] = speed[2];

        delete []speed;
    }
    ser_front.send(stop);
    ser_back.send(stop);
    std::cout<<"线程2--thread2_pathplaning exited"<<std::endl;
}

// 线程3
void save_data_thread() {
    timeb t_save_start,t_save_end;
    cv::VideoWriter video_writer;
    video_writer.open("/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/forward.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 16, cv::Size(640, 480));
    while (true) {
        // 获取队列中的数据
        std::unique_lock<std::mutex> image_lock_save(image_mutex_save);
        save_cv.wait(image_lock_save, [](){ return !imageinfo_queue_save.empty() || is_exit; });
        if (is_exit && imageinfo_queue_save.empty()) {
            break;// 如果主线程已经退出并且队列为空，则退出循环
        }

        imageinfo imginfo = imageinfo_queue_save.front();//读取数据
        imageinfo_queue_save.pop();
        image_lock_save.unlock();// 释放锁，允许主线程往队列中添加新的数据


        ftime(&t_save_start);

        // 保存数据到硬盘中
        float predict_peo[3] = {0,0,0};
        jointsrender(imginfo.forwardrgb, imginfo.joints, imginfo.maxval, 1.0 / 10, predict_peo, imginfo.frameid);//渲染关节点
        video_writer.write(imginfo.forwardrgb);//保存视频

        // stringstream ss;
        // ss << setw(10) << setfill('0') << imginfo.frameid;
        // string rgb_file = "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/RGB/" + ss.str() + ".png";
        // imwrite(rgb_file, imginfo.forwardrgb);

        // stringstream ss;
        // ss << setw(10) << setfill('0') << imginfo.frameid;
        // string depth_file = "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/depth/" + ss.str() + ".png";
        // imwrite(depth_file, imginfo.forwarddepth);
        // std::cout << "线程3--Saving frame with id " << imginfo.frameid << " to disk..." << std::endl;

        ftime(&t_save_end);
        double save_time = t_save_end.time + t_save_end.millitm * 1e-3 - (t_save_start.time + t_save_start.millitm * 1e-3);
        // std::cout << "线程3--保存耗时save_time =  " <<dec<< save_time*1000 << std::endl;
    }
    video_writer.release();
    std::cout<<"线程3--thread3_savedata exited"<<std::endl;
}

unsigned char running = 1;
static void sig_handle(int signo)
{
    printf("program exit, [%s,%s] Receive SIGNAL %d ====== \r\n", __FILE__, __func__, signo);
    running = 0;
    sleep(2);
    std::exit(1);  // 有可能析构还没结束 就推出了！不行哦
}
void test_move(){
    std::string portname_bc = "/dev/ttyTHS0";
    std::string portname_ad = "/dev/ttyTCU0";
    int baudrate = 115200;

    serial::serialsend ser_bc(portname_bc, baudrate);
    serial::serialsend ser_ad(portname_ad, baudrate);

    float speed_front[3] = {0.3,0,0};
    float speed_back[3] = {-0.3,0,0};

    float speed_left[3] = {0,0.3,0};
    float speed_right[3] = {0,-0.3,0};


    float speed_leftFront[3] = {0.15,0.15,0};
    float speed_rightback[3] = {-0.215,-0.215,0};

    float speed_rightFront[3] = {0.215,-0.215,0};
    float speed_leftback[3] = {-0.15,0.15,0};

    float stop[3] = {0,0,0};

    float rotate[3] = {0,0,0.8};

    int time_cur = 1;
    char input;
    while (1)
    {
        std::cout<< "请输入运动方向:"<< std::endl;
        std::cin>>input;
        int flag = input-'0';
        switch(flag){
            case 0: 
                ser_ad.send(rotate);
                ser_bc.send(rotate);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 8: 
                ser_ad.send(speed_front);
                ser_bc.send(speed_front);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 2: 
                ser_ad.send(speed_back);
                ser_bc.send(speed_back);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 4: 
                ser_ad.send(speed_left);
                ser_bc.send(speed_left);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 6: 
                ser_ad.send(speed_right);
                ser_bc.send(speed_right);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 7: 
                ser_ad.send(speed_leftFront);
                ser_bc.send(speed_leftFront);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 3: 
                ser_ad.send(speed_rightback);
                ser_bc.send(speed_rightback);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 9: 
                ser_ad.send(speed_rightFront);
                ser_bc.send(speed_rightFront);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 1: 
                ser_ad.send(speed_leftback);
                ser_bc.send(speed_leftback);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 5: 
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            default:
                continue;
        }
        
    }
}

int main(int argc, char *argv[]) {
    signal(SIGINT, sig_handle);
	//创建子线程
    std::thread thread1_generatemap(generate_map_thread);
    std::thread thread2_pathplaning(path_planing_thread);
    std::thread thread3_savedata(save_data_thread);
   
    // 雷达初始化
    // uint8_t type = 0x0;  // 0x0,/**< serial type.*/
    // int model = 1;       // ORADAR_MS200 = 1
    //std::unique_ptr<radar_reader> radarreader(new radar_reader("/dev/ttyACM0", 230400, type, model));
    radar_reader radarreader("/dev/ttyACM0", 230400);

	HumanPoseEstimator model(initializeSampleParams());
	sample::gLogInfo << "主线程--Loading and running a GPU inference engine" << std::endl;
	if (!model.load()){
		printf("主线程--load model failed");
	}
	samplesCommon::BufferManager buffers(model.mEngine);
	auto context = SampleUniquePtr<nvinfer1::IExecutionContext>(model.mEngine->createExecutionContext());
	if (!context){
		printf("主线程--load model pointer failed");
	}
    // 获取关节点个数
	std::vector<std::string>::iterator iter = std::find(model.mParams.outputTensorNames.begin(),model.mParams.outputTensorNames.end(), "pred");
	int ind = iter - model.mParams.outputTensorNames.begin();
	int nJoints = model.mOutputDims[ind].d[1];                      // 关节点的个数
	std::vector<cv::Point2f> joints(nJoints);                       // 存储关节点的位置信息
	std::vector<double> maxval(nJoints);                            // 关节点的置信度

    /*启动 forward 相机*/
	if (!astra_reader.init(0, 2, 640, 480)){
		std::cout << "主线程--Astra前相机未连接,请检查!" << std::endl;
	}
    std::cout<<"主线程--前向相机已启动..."<<std::endl;
    /*启动 back 相机*/
	if (!astra_reader_back.init(1, 0, 320, 240)){
		std::cout << "主线程--Astra后相机未连接,请检查!" << std::endl;
	}
    std::cout<<"主线程--后向相机已启动..."<<std::endl;

    int cycle_times = 1;
    int save_queue_length;
    timeb t_capture_start,t_capture_end;
    timeb t_end;
 
    
    cv::Mat img_rgb, depth_forward, backrgb, depth_back;
    bool IsCheck2CameraImg = true;

    while(running) {
       //test_move();

        ftime(&t_capture_start);

        // // 雷达
        // if(!radarreader.radar_read()) {
        //     std::cout<< "failed read radar data!"<< std::endl;
        // }
        // full_scan_data_st scan_data = radarreader.full_scan_data;
        // for (int i = 0; i < scan_data.vailtidy_point_num; i++)
        // {
        //     printf("[%d: %f, %f] \n", i, (scan_data.data[i].distance * 0.001), scan_data.data[i].angle);
        // }

        // 读取相机图像
        // 获取前向rgb图
        if(!astra_reader.read_rgb(img_rgb)){
            std::cout<<"read img_rgb failed!!"<<std::endl;
        }

        // 推理
        cv::Mat img_rgb_infer;
        cv::resize(img_rgb, img_rgb_infer, cv::Size(256, 192), 0.0, 0.0, cv::INTER_CUBIC);
        // 推理获取关节点坐标和置信度,关节点像素坐标保存于joints
        if (!model.infer(context, buffers, img_rgb_infer, joints, maxval)){
            printf("主线程--inference failed");
        }
        ftime(&t_capture_end);
        double capture_time = t_capture_end.time + t_capture_end.millitm * 1e-3 - (t_capture_start.time + t_capture_start.millitm * 1e-3);
        std::cout << "主线程--捕获RGB图+推理耗时capture_time =  " << capture_time*1000 << std::endl;

        // 获取前向深度图
        if(!astra_reader.read_depth(depth_forward)){
            std::cout<<"read depth_forward failed!!"<<std::endl;
            return false;
        }

        // 获取后向rgb图
        if(!astra_reader_back.read_rgb(backrgb)){
            std::cout<<"read backrgb failed!!"<<std::endl;
            return false;
        }
        cv::resize(backrgb, backrgb, cv::Size(640, 480), 0.0, 0.0, cv::INTER_CUBIC);  //cv::Size(320, 240)
        
        // 获取后向深度图
        if(!astra_reader_back.read_depth(depth_back)){
            std::cout<<"read depth_back failed!!"<<std::endl;
            return false;
        }
        cv::resize(depth_back, depth_back, cv::Size(640, 480), 0.0, 0.0, cv::INTER_CUBIC);  //cv::Size(320, 240)

        // 保存
        // if(IsCheck2CameraImg){
        //     cv::imwrite("img_rgb_forward.png", img_rgb);
        //     cv::imwrite("backrgb.png", backrgb);
        //     cv::imwrite("depth_forward.png", depth_forward);
        //     cv::imwrite("depth_back.png", depth_back);
        // }


        //获取关节点三维坐标
        std::vector<cv::Point3f> joints_coord = humandetect::get_joints_coord_bydepth(joints,maxval,depth_forward);
        imageinfo imginfo(cycle_times,get_time_now(),img_rgb,depth_forward,backrgb,depth_back,joints,joints_coord,maxval);

        std::unique_lock<std::mutex> image_lock_use(image_mutex_use);//放到使用队列中
        imageinfo_queue_use.push(imginfo);
        // std::cout << "主线程--use队列长度：" << imageinfo_queue_use.size()<< std::endl;//只会是1才对
        image_lock_use.unlock();
        image_cv.notify_one();                      // 条件变量，通知线程1，由阻塞态变为就绪态

        std::unique_lock<std::mutex> image_lock_save(image_mutex_save);//放到保存队列中
        imageinfo_queue_save.push(imginfo);
        save_queue_length = imageinfo_queue_save.size();
        image_lock_save.unlock();
        save_cv.notify_one();
        
        if(save_queue_length >= 100){std::cerr << "主线程--警告:待保存队列长度大于100" << std::endl;}
        cycle_times++;

        ftime(&t_end);
        double total_time = t_end.time + t_end.millitm * 1e-3 - (t_capture_start.time + t_capture_start.millitm * 1e-3);
        // std::cout << "主线程--总帧数 =  " << cycle_times << std::endl;
        // std::cout << "主线程--总耗时total_time =  " <<dec<< total_time*1000 << std::endl;

        if(kbhit()) {
            int key = getch();
            if(key == KEY_ESC) {//按ESC键退出
                break;
            }
        }
        sleep(1);
    }
    radarreader.save_fulldata(); // save到txt中，只有一针
    
	std::unique_lock<std::mutex> image_lock_use(image_mutex_use);//is_exit只有这里操作，所以应该加不加lock都行
    is_exit = true;
    image_lock_use.unlock();
    image_cv.notify_one();
    map_cv.notify_one();
    save_cv.notify_one();

    // 等待后台线程
    thread1_generatemap.join();
    thread2_pathplaning.join();
    thread3_savedata.join();

    std::cout<<"主线程--主线程结束，总帧数 = "<< cycle_times <<std::endl;
    return 0;
}