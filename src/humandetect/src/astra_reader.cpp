#include "astra_reader.hpp"

std::vector<std::string> AstraReader::deviceUrl_list;  // 静态变量在.cpp中初始化。


AstraReader::AstraReader(trans_param astra2world) : 
    fov{45.5 / 180 / M_PI, 58.4 / 180 / M_PI},
    // focus{600.552869 / 480, 601.567238 / 640},  //-- {fy/480,fx/640}(在width*height = 640*480下)
    // center{245.724830 / 480, 317.579310 / 640}, //-- {v0/480,u0/640}(在width*height = 640*480下)
    focus{601.567238, 600.552869},  //-- {fx,fy}(在width*height = 640*480下)
    center{317.579310, 245.724830}, //--  {u0,v0}(在width*height = 640*480下)
    
    is_lidar(false),
    max_depth(4000),  //单位毫米
    min_depth(20),
    
    cos_theta(cos(astra2world.Theta)),
    sin_theta(sin(astra2world.Theta)),
    Tx(astra2world.TranslationX),
    Ty(astra2world.TranslationY)
{
    
    openni::OpenNI::initialize();
}

AstraReader::~AstraReader()
{
    cap.release();
    depth_stream.destroy();
    dev.close();
    openni::OpenNI::shutdown();
}

void AstraReader::getDeviceUri() {
    openni::Array<openni::DeviceInfo> deviceList;
    openni::OpenNI::enumerateDevices(&deviceList);
    for(int i = 0; i<deviceList.getSize(); ++i) {
        const openni::DeviceInfo& info = deviceList[i];
        const char* uri = info.getUri();
        deviceUrl_list.emplace_back((uri));
    }
    
}
/*
    捕捉rgb流 和 深度流,
*/
bool AstraReader::init(int dev_index,int cap_index,int width, int hight)
{   /*--------------------------捕捉视频流------------------------------------*/
    cap = cv::VideoCapture(cap_index);                      // 
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, hight);        // 高
	cap.set(cv::CAP_PROP_FRAME_WIDTH, width);         // 宽
	cap.set(cv::CAP_PROP_FPS, 30);                  // 帧率
    /*--------------------------捕捉深度流------------------------------------*/

    /* ---------获取相机的uri---------- */
    if(deviceUrl_list.empty()) {       // 没有获取url就获取一下，给静态变量赋值
        getDeviceUri();
    }
    

    /****根据uri打开不同的相机******/
    // auto rc = dev.open(openni::ANY_DEVICE);
    auto rc = dev.open(deviceUrl_list[dev_index].c_str());
    if (rc != openni::STATUS_OK)
    {
        printf("open error! error num:%d", rc);
        return false;
    }
    if (dev.getSensorInfo(openni::SENSOR_DEPTH) != NULL)        // 函数获取设备中的深度传感器信息
    {
        rc = depth_stream.create(dev, openni::SENSOR_DEPTH);    // 创建 depth_stream 对象并调用其 create() 函数打开深度传感器流
        if (rc != openni::STATUS_OK)
        {
            printf("depth_stream created error!");
            return false;
        }
        // 设置分辨率，FPS，像素格式
        openni::VideoMode mode;
        // mode.setResolution(640, 480);
        mode.setResolution(width, hight);
        mode.setFps(30);
        mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
        depth_stream.setVideoMode(mode);            // 把模式设置到 depth_stream 中去
        depth_stream.setMirroringEnabled(false);    // 是否镜像
    }
    // 启动深度相机的深度流
    rc = depth_stream.start();
    if (rc != openni::STATUS_OK)
    {
        printf("depth_stream started error!");
        return false;
    }
    // 设置深度图像和彩色图像的配准模式
    rc = dev.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    if (rc != openni::STATUS_OK)
    {
        return false;
    }
    return true;
}


bool AstraReader::read_rgb(cv::Mat &img)
{
    if (!cap.read(img))
    {
        return false;
    }
    return true;
}

bool AstraReader::read_depth(cv::Mat &img)
{
    int changedStreamDummy;
    openni::VideoStream *p_stream = &depth_stream;
    auto rc = openni::OpenNI::waitForAnyStream(&p_stream, 1, &changedStreamDummy, 1000);
    if (rc != openni::STATUS_OK)
    {
        return false;
    }

    openni::VideoFrameRef frame;
    rc = depth_stream.readFrame(&frame);
    if (rc != openni::STATUS_OK)
    {
        return false;
    }

    openni::PixelFormat pformat = frame.getVideoMode().getPixelFormat();
    if(pformat != openni::PIXEL_FORMAT_DEPTH_1_MM && pformat != openni::PIXEL_FORMAT_DEPTH_100_UM)
    {
        return false;
    }

    openni::DepthPixel* pDepth = (openni::DepthPixel*)frame.getData();
    img = cv::Mat(frame.getHeight(), frame.getWidth(), CV_16UC1, (void*)frame.getData());
    return true;
}

bool AstraReader::depth2point(cv::Mat &depth_img, std::vector<cv::Point2f> &joints,
    std::vector<cv::Point3f> &points, std::vector<double> maxval)
{
    // joints需要转换到深度图尺度大小
    int count, range = 2;
    float thres = CONFIDENCE_THRES;
    cv::Point2f c;
    cv::Mat crop, mask;
    cv::Mat avg_depth(1, 1, CV_32FC1, cv::Scalar(1e-4));
    std::vector<cv::Point2f> joints_(joints.size());
    for (int i = 0; i < joints.size(); i++)
    {
        joints_[i].x = joints[i].x * depth_img.cols;
		joints_[i].y = joints[i].y * depth_img.rows;
        
        c = joints_[i];
        if (c.x+range < 0 || c.x-range > depth_img.cols-1 || c.y+range < 0
            || c.y-range > depth_img.rows-1 || maxval[i] <= thres)
        {
            points[i].x = -1;
            points[i].y = -1;
            points[i].z = -1;
            continue;
        }

        if (c.x - range < 0)
            c.x = range;
        else if (c.x + range > depth_img.cols - 1)
            c.x = depth_img.cols - range - 1;

        if (c.y - range < 0)
            c.y = range;
        else if (c.y + range > depth_img.rows - 1)
            c.y = depth_img.rows - range - 1;

        points[i].z = 0;
        count = 0;
        // for (int dx = -range; dx < range + 1; dx++)
        // {
            // for (int dy = -range; dy < range + 1; dy++)
            // {
                // if (depth_img.at<uint16_t>((int)c.y + dy, (int)c.x + dx) <= 0)
                    // continue;
                // count++;
                // points[i].z += (depth_img.at<uint16_t>((int)c.y + dy, (int)c.x + dx) - points[i].z) / count;
            // }
        // }

        crop = depth_img(cv::Rect(c.x - range, c.y - range, 2 * range + 1, 2 * range + 1));
        cv::threshold(crop, mask, 0, 1, cv::THRESH_BINARY);
        cv::max(cv::Mat(1, 1, CV_32FC1, cv::mean(mask)), cv::Mat(1, 1, CV_32FC1, cv::Scalar(1e-4)), avg_depth);
        cv::divide(cv::mean(crop), avg_depth, avg_depth, 1, CV_32FC1);
        points[i].z = avg_depth.at<float>(0, 0);
        points[i].z = (points[i].z>1) ? points[i].z : 0;

        if (is_lidar)
        {
            float phi = M_PI / 2 - fov[0] / 2 + fov[0] / (depth_img.rows - 1) * joints_[i].x;
            float theta = M_PI / 2 - fov[1] / 2 + fov[1] / (depth_img.cols - 1) * joints_[i].y;
            points[i].y = points[i].z * sin(phi) * cos(theta);
            points[i].x = points[i].z * cos(phi);
            points[i].z = points[i].z * sin(phi) * sin(theta);
        }
        else
        {
            float fx = focus[1];//focus[0] * depth_img.rows;
            float fy = focus[0];//focus[1] * depth_img.cols;
            float cx = center[1];//center[0] * depth_img.rows;
            float cy = center[0];//center[1] * depth_img.cols;
            points[i].x = (depth_img.cols - joints_[i].y - cx) / fx * points[i].z;
            points[i].y = (joints_[i].x - cy) / fy * points[i].z;
        }
    }
    return true;    
}

bool AstraReader::GenePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr AstraCloud){
    /*先读出深度图*/
    int changedStreamDummy;
    openni::VideoStream *p_stream = &depth_stream;
    auto rc = openni::OpenNI::waitForAnyStream(&p_stream, 1, &changedStreamDummy, 1000);
    if (rc != openni::STATUS_OK){
        return false;
    }

    openni::VideoFrameRef frame;
    rc = depth_stream.readFrame(&frame);
    if (rc != openni::STATUS_OK){
        return false;
    }

    openni::PixelFormat pformat = frame.getVideoMode().getPixelFormat();
    if(pformat != openni::PIXEL_FORMAT_DEPTH_1_MM && pformat != openni::PIXEL_FORMAT_DEPTH_100_UM){
        return false;
    }
    openni::DepthPixel* pDepth = (openni::DepthPixel*)frame.getData();
    int width = frame.getWidth();
    int height = frame.getHeight();

    /*深度图转为PCL点云*/
	AstraCloud->width = width;
	AstraCloud->height = height;
	AstraCloud->is_dense = false;
	AstraCloud->points.resize(width*height);
    //分辨率缩放
    float fdx = focus[0];
    float fdy = focus[1];
    float u0 = center[0];
    float v0 = center[1];

    for (int v = 0; v < height; ++v){
		for (int u = 0; u < width; ++u){
            int index = v * width + u;
			uint16_t depth = pDepth[index];
			if (depth <= 0 || depth < min_depth || depth > max_depth){//无效值
                AstraCloud->points[index].x = 0;
                AstraCloud->points[index].y = 0;
                AstraCloud->points[index].z = 0;
            }
            else{
                float tx = (u - u0) / fdx;
	            float ty = (v - v0) / fdy;
                AstraCloud->points[index].x = depth * tx / 1000.0;
                AstraCloud->points[index].y = -depth * ty / 1000.0;
                AstraCloud->points[index].z = -depth / 1000.0;
                
            }
		}
	}
    return true;
}


/*
    深度图计算障碍物位置的函数；
    pixels用于保存障碍物对应像素坐标
    给定obsheight是相机坐标系下，障碍物探测高度（y轴值）
    obs_mindist用于控制障碍物稀疏程度
*/
Position * AstraReader::depth2obs_astra(cv::Mat &depthframe , std::vector<cv::Point2f> &pixels ,  int * obs_nums ,const float obsheight ,const float obs_mindist,const float valid_obs_dist){
    /*先读出深度数据*/
    uint16_t *pDepth = (uint16_t *)depthframe.data;
    int width = depthframe.cols;
    int height = depthframe.rows;
    float (*obs_position)[2] = new Position[width];
    std::vector<float> previous_obs(2);
    *obs_nums = 0;
    //分辨率缩放
    float fx = focus[0];
    float fy = focus[1];
    float u0 = center[0];
    float v0 = center[1];

    for (int u = 0; u < width; u=u+4){
        for (int v = height-1; v >= height/2; v=v-4){
            int index = v * width + u;
			uint16_t depth = pDepth[index];
			if (depth <= 0 || depth < min_depth || depth > max_depth){//无效值
                continue;
            }
            else{
                float tx = (u - u0) / fx;
	            float ty = (v - v0) / fy;
                float y = depth * ty/1000.0;
                if(y>obsheight-0.005 && y<obsheight+0.01){
                    float x = depth * tx/1000.0;
                    float z = depth/1000.0;
                    float obs_dist = *obs_nums==0 ? INFINITY : sqrt(pow(z-previous_obs[0],2)+pow(-x-previous_obs[1],2));
                    if(obs_dist < obs_mindist){
                        break;//相邻障碍物点太近的不需要，从而减少障碍物数目
                    }
                    else if(sqrt(pow(z,2)+pow(-x,2)) > valid_obs_dist){
                        break;//离机器人太远的不需要，从而减少障碍物数目
                    }
                    else{
                        pixels[*obs_nums].x = u;pixels[*obs_nums].y = v;
                        previous_obs[0] = z;previous_obs[1] = -x;
                        obs_position[*obs_nums][0] = previous_obs[0] * this->cos_theta - previous_obs[1] * this->sin_theta + this->Tx;//转换至机器人坐标系
                        obs_position[*obs_nums][1] = previous_obs[0] * this->sin_theta + previous_obs[1] * this->cos_theta + this->Ty;
                        *obs_nums = *obs_nums+1;
                        break;//找到该方向最近障碍物就退出，进入下一列
                    }
                }
            }
        }
    }
    if(*obs_nums == 0){
        std::cout << "当前没有障碍物" <<std::endl;   
    }
    // std::cout << "astra障碍物数目 = " << *obs_nums <<std::endl;   
    return obs_position;
}