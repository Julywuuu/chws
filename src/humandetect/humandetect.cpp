#include "humandetect.h"

// trans_param test_trans = {0,0,0};
// AstraReader intrinsics(test_trans);
extern AstraReader astra_reader;

//输入关节点位置，返回对应点云坐标
std::vector<cv::Point3f> humandetect::get_joints_coord(std::vector<cv::Point2f> joints,std::vector<double> maxval,
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    // joints需要转换到RGB图尺度大小
    float thres = CONFIDENCE_THRES;
    std::vector<cv::Point3f> joints_coord(joints.size());
    //由于(1)鲁棒性考虑(2)某一个像素对应点云坐标可能为缺失
    //故计算joints对应像素点及其上下左右四个像素点中所有有效值的均值
    for (int i = 0; i < joints.size(); i++){
        if(maxval[i] <= thres){//小于置信度阈值，该点设为无效点
            joints_coord[i].x = 0;joints_coord[i].y = 0;joints_coord[i].z = 0;
            continue;
        }
        int pixelx = round(joints[i].x * WIDTH)-1;//减1符合C++的图像表示
        int pixely = round(joints[i].y * HEIGHT)-1;
        std::vector<cv::Point2f> points(5);
        points[0].x=pixelx; points[0].y=pixely-1;
        points[1].x=pixelx-1; points[1].y=pixely;
        points[2].x=pixelx; points[2].y=pixely;
        points[3].x=pixelx+1; points[3].y=pixely;
        points[4].x=pixelx; points[4].y=pixely+1;
        int invalid_nums = 0;
        float sumx = 0;float sumy = 0;float sumz = 0;
        for(int j = 0;j < points.size(); j++){
            if((points[j].x>=WIDTH)||(points[j].x<0)||(points[j].y>=HEIGHT)||(points[j].y<0)){
                invalid_nums++;//超出范围的点无效
                continue;
            }
            int index = points[j].y*WIDTH + points[j].x;
            if(cloud->points[index].x==0&&cloud->points[index].y==0&&cloud->points[index].z==0){
                invalid_nums++;//点云缺失的点无效
                continue;
            }
            sumx += cloud->points[index].x;
            sumy += cloud->points[index].y;
            sumz += cloud->points[index].z;
        }
        if(invalid_nums == points.size()){//虽然该点可置信，但该店及其周围点均无有效深度
            joints_coord[i].x = 0;joints_coord[i].y = 0;joints_coord[i].z = 0;
            continue;
        }
        joints_coord[i].x = sumx/(points.size()-invalid_nums);
        joints_coord[i].y = sumy/(points.size()-invalid_nums);
        joints_coord[i].z = sumz/(points.size()-invalid_nums);
    }
    
    return joints_coord;
};



//使用深度图，输入关节点位置，返回对应点云坐标
//默认使用intel相机作为识别人体的相机
std::vector<cv::Point3f> humandetect::get_joints_coord_bydepth(std::vector<cv::Point2f> joints,std::vector<double> maxval,
cv::Mat depthframe){
    // joints需要转换到RGB图尺度大小
    float thres = CONFIDENCE_THRES;
    std::vector<cv::Point3f> joints_coord(joints.size());
    uint16_t *pDepth = (uint16_t *)depthframe.data;//深度数据
    float fx = astra_reader.focus[0];
    float fy = astra_reader.focus[1];
    float u0 = astra_reader.center[0];
    float v0 = astra_reader.center[1];
    //由于(1)鲁棒性考虑(2)某一个像素对应点云坐标可能为缺失
    //故计算joints对应像素点及其上下左右四个像素点中所有有效值的均值
    for (int i = 0; i < joints.size(); i++){
        if(maxval[i] <= thres){//小于置信度阈值，该点设为无效点
            joints_coord[i].x = 0;joints_coord[i].y = 0;joints_coord[i].z = 0;
            continue;
        }
        int pixelx = round(joints[i].x * WIDTH)-1;//减1符合C++的图像表示
        int pixely = round(joints[i].y * HEIGHT)-1;
        std::vector<cv::Point2f> points(5);
        points[0].x=pixelx; points[0].y=pixely-1;
        points[1].x=pixelx-1; points[1].y=pixely;
        points[2].x=pixelx; points[2].y=pixely;
        points[3].x=pixelx+1; points[3].y=pixely;
        points[4].x=pixelx; points[4].y=pixely+1;
        int invalid_nums = 0;
        float sumx = 0;float sumy = 0;float sumz = 0;
        for(int j = 0;j < points.size(); j++){
            if((points[j].x>=WIDTH)||(points[j].x<0)||(points[j].y>=HEIGHT)||(points[j].y<0)){
                invalid_nums++;//超出范围的点无效
                continue;
            }
            int index = points[j].y*WIDTH + points[j].x;
			uint16_t depth = pDepth[index];
			if (depth <= 0 || depth < astra_reader.min_depth || depth > astra_reader.max_depth){
                invalid_nums++;//深度无效的点
                continue;
            }
            float tx = (points[j].x - u0) / fx;
            float ty = (points[j].y - v0) / fy;
            float x_coord = depth * tx/1000.0;
            float y_coord = -depth * ty/1000.0;
            float z_coord = -depth/1000.0;
            sumx += x_coord;sumy += y_coord;sumz += z_coord;
        }
        if(invalid_nums == points.size()){//虽然该点可置信，但该店及其周围点均无有效深度
            joints_coord[i].x = 0;joints_coord[i].y = 0;joints_coord[i].z = 0;
            continue;
        }
        joints_coord[i].x = sumx/(points.size()-invalid_nums);
        joints_coord[i].y = sumy/(points.size()-invalid_nums);
        joints_coord[i].z = sumz/(points.size()-invalid_nums);
    }
    
    return joints_coord;  
};


//输入关节点点云坐标，返回人体位置、朝向
float * humandetect::get_human_state(std::vector<cv::Point3f> joints_coord){
    float * peo = new float[3];
    cv::Point3f invalid_point;invalid_point.x = 0;invalid_point.y = 0;invalid_point.z = 0;
    //通过深度值判断关节点是否有效(真正落在人体关节上),三个点的深度值极差小于0.5认为有效
    float deepthreshold = 0.5;
    cv::Point3f point2 = joints_coord[2];//右髋
    cv::Point3f point6 = joints_coord[6];//小腹
    cv::Point3f point3 = joints_coord[3];//左髋
    if(point2!=invalid_point && point6!=invalid_point && point3!=invalid_point){
        //均为有效点
        //均有效，但不一定真正落在人体关节上
        if(fabs(point2.z-point6.z)>deepthreshold || fabs(point2.z-point3.z)>deepthreshold || fabs(point6.z-point3.z)>deepthreshold){
            //有部分点未落在关节上时，以point6（最中间）为准
            peo[0] = -point6.z;
            peo[1] = -point6.x;
            peo[2] = INFINITY;
        }
        else{
            peo[0] = -(point2.z+point6.z+point3.z)/3;
            peo[1] = -(point2.x+point6.x+point3.x)/3;
            float temp_theta = M_PI/2 + atan2(-point2.x + point3.x, -point2.z + point3.z);
            peo[2] = atan2(sin(temp_theta), cos(temp_theta));
        }
    }
    else if(point6!=invalid_point){
        //point6有效
        //此时point2，3至少一个失效，所以无法获取朝向
        peo[0] = -point6.z;
        peo[1] = -point6.x;
        peo[2] = INFINITY;
    }
    else if(point2!=invalid_point && point3!=invalid_point){
        //point2，3有效
        if(fabs(point2.z-point3.z)>deepthreshold){
            peo[0] = INFINITY;peo[1] = INFINITY;peo[2] = INFINITY;
        }
        else{
            peo[0] = -(point2.z+point3.z)/2;
            peo[1] = -(point2.x+point3.x)/2;
            float temp_theta = M_PI/2 + atan2(-point2.x + point3.x, -point2.z + point3.z);
            peo[2] = atan2(sin(temp_theta), cos(temp_theta));
        }
    }
    else{//其他情况,返回[INFINITY,INFINITY,INFINITY]
        peo[0] = INFINITY;peo[1] = INFINITY;peo[2] = INFINITY;
    }
    
    //如何滤除异常值
    //由于关节点在视角不好的情况下可能发生对称的左右序号交换
    //所以可以多重验证(如加上肩膀的点构成的向量)/记录相邻帧间的人体朝向变化,并限制之
    //但如果可以采集到肩膀一般视角还算好,相邻帧的变化限制阈值不好确定,所以先不做判断试一试

    //另一种策略：不使用多个关节点计算朝向，只使用某个有效点+有效点左边像素点计算朝向
    //这样的问题是人的前后不分，不过影响还好

    //无论人体前后，只计算人体这一平面法线朝着机器人这一侧的方向角，那么peo[2]一定是[-pi,-pi/2)或(pi/2,pi]之间的数
    if(peo[2]!=INFINITY){//这样角度计算鲁棒了，但是由于人在走路过程中关节点不是平行的，跨步时会变化，所以该角度只能在速度趋于0时使用
        if(peo[2]>=-M_PI/2 && peo[2]<0){//(或许可以考虑使用均值滤波？)
            peo[2] = peo[2] + M_PI;
        }
        if(peo[2]>=0 && peo[2]<=M_PI/2){
            peo[2] = peo[2] - M_PI;
        }
    }
    return peo;
};

//使用膝关节连线中点作为人体位置；好处是一方面人体运动时膝关节先于髋关节运动，用膝关节表示人体位置或许跟随效果更好；
//另一方面，膝关节位置更低，相对髋关节不容易失去人体位置信息（踝关节容易误识别到地面上，故不考虑）
//或者考虑始终以靠近机器人的膝关节位置作为人体位置
float * humandetect::get_human_state_knee(std::vector<cv::Point3f> joints_coord){
    float * peo = new float[3];
    cv::Point3f invalid_point;invalid_point.x = 0;invalid_point.y = 0;invalid_point.z = 0;
    cv::Point3f point1 = joints_coord[1];//右膝关节
    cv::Point3f point4 = joints_coord[4];//左膝关节
    cv::Point3f point2 = joints_coord[2];//右髋
    cv::Point3f point3 = joints_coord[3];//左髋
    // if(point1!=invalid_point && point4!=invalid_point){//计算人体位置
    //     //均为有效点
    //     peo[0] = -(point1.z+point4.z)/2;
    //     peo[1] = -(point1.x+point4.x)/2;
    // }
    if(point1!=invalid_point && point4!=invalid_point){//计算人体位置
        //均为有效点
        float dist1 = point1.z*point1.z+point1.x*point1.x;
        float dist4 = point4.z*point4.z+point4.x*point4.x;
        if(dist1 < dist4){
            peo[0] = -point1.z;
        }
        else{
            peo[0] = -point4.z;        
        }
        peo[1] = -(point1.x+point4.x)/2.0;
    }
    else{//其他情况,返回[INFINITY,INFINITY,INFINITY]
        peo[0] = INFINITY;peo[1] = INFINITY;
    }
    if(point2!=invalid_point && point3!=invalid_point){//计算人体朝向
        //均为有效点
        float temp_theta = M_PI/2 + atan2(-point2.x + point3.x, -point2.z + point3.z);
        peo[2] = atan2(sin(temp_theta), cos(temp_theta));
    }
    else{//
        peo[2] = INFINITY;
    }

    //无论人体前后，只计算人体这一平面法线朝着机器人这一侧的方向角，那么peo[2]一定是[-pi,-pi/2)或(pi/2,pi]之间的数
    if(peo[2]!=INFINITY){//这样角度计算鲁棒了，但是由于人在走路过程中关节点不是平行的，跨步时会变化，所以该角度只能在速度趋于0时使用
        if(peo[2]>=-M_PI/2 && peo[2]<0){//(或许可以考虑在主函数中使用均值滤波？)
            peo[2] = peo[2] + M_PI;
        }
        if(peo[2]>=0 && peo[2]<=M_PI/2){
            peo[2] = peo[2] - M_PI;
        }
    }
    return peo;
};


//最小二乘拟合平面方程  z = a0x + a1y + a2
std::vector<double> humandetect::FittingPlaneZ(std::vector<cv::Point3f> peo_plane,int pixels_nums){
    std::vector<std::vector<double>> mat3x4(3, std::vector<double>(4, 0));
    for (int i=0;i<pixels_nums;i++) {
        mat3x4[0][0] += peo_plane[i].x * peo_plane[i].x;
        mat3x4[0][1] += peo_plane[i].x * peo_plane[i].y;
        mat3x4[0][2] += peo_plane[i].x;
        mat3x4[1][1] += peo_plane[i].y * peo_plane[i].y;
        mat3x4[1][2] += peo_plane[i].y;

        // 3x1 方程组值
        mat3x4[0][3] += peo_plane[i].x * peo_plane[i].z;
        mat3x4[1][3] += peo_plane[i].y * peo_plane[i].z;
        mat3x4[2][3] += peo_plane[i].z;
    }

    Eigen::MatrixXd A(3, 3);
    A <<mat3x4[0][0], mat3x4[0][1], mat3x4[0][2],
        mat3x4[0][1], mat3x4[1][1], mat3x4[1][2],
        mat3x4[0][2], mat3x4[1][2], (double)pixels_nums;
    Eigen::MatrixXd B(3, 1);
    B << mat3x4[0][3], mat3x4[1][3], mat3x4[2][3];

    Eigen::MatrixXd x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
    return { x(0), x(1), -1.0, x(2) };//x(0)x + x(1)y -z + x(2) = 0
};


//拟合髋关节附近人体平面，从而获取人体朝向
float humandetect::get_human_direction(std::vector<cv::Point2f> joints,std::vector<cv::Point3f> joints_coord,cv::Mat depthframe,
std::vector<cv::Point2f> &pixels,int * pixels_nums){
    float peo_direction;
    *pixels_nums = 0;
    cv::Point3f invalid_point;invalid_point.x = 0;invalid_point.y = 0;invalid_point.z = 0;
    uint16_t *pDepth = (uint16_t *)depthframe.data;//深度数据
    float fx = astra_reader.focus[0];
    float fy = astra_reader.focus[1];
    float u0 = astra_reader.center[0]; 
    float v0 = astra_reader.center[1];

    cv::Point2f pixel2,pixel3;
    cv::Point3f point2 = joints_coord[2];//右髋
    cv::Point3f point3 = joints_coord[3];//左髋
    if(point2==invalid_point || point3==invalid_point){//髋关节点无效，朝向无法获取
        peo_direction = INFINITY;
        return peo_direction;
    }
    else{
        pixel2.x = round(joints[2].x * WIDTH)-1;pixel2.y = round(joints[2].y * HEIGHT)-1;
        pixel3.x = round(joints[3].x * WIDTH)-1;pixel3.y = round(joints[3].y * HEIGHT)-1;

        float min_z = ((-point2.z - point3.z)/2.0 - 0.25)*1000;//用于去除背景,乘1000化为深度图下对应尺度,-point2.z是坐标系转换
        float max_z = ((-point2.z - point3.z)/2.0 + 0.25)*1000;

        //在深度图中取出关节点2，3所在的矩形区域（向上取，上身在运动过程中较为鲁棒）
        int length = abs(pixel2.x-pixel3.x);//矩形边长
        int startpixel_x = std::max(pixel2.x,pixel3.x) - length;
        int startpixel_y = std::max(pixel2.y,pixel3.y) - length;

        std::vector<cv::Point3f> peo_plane(length*length);
        for(int u = startpixel_x; u < startpixel_x+length; u++){
            for(int v = startpixel_y; v < startpixel_y+length; v++){
                if(u>=WIDTH || u<0 || v>=HEIGHT || v<0 ){//避免取的矩形区域超出深度图像素范围 
                    continue;
                }
                int index = v * WIDTH + u;
                uint16_t depth = pDepth[index];
                if (depth <= 0 || depth < astra_reader.min_depth || depth > astra_reader.max_depth || depth < min_z || depth > max_z){
                    continue;//去除矩形区域中的背景像素点/无效深度点，得到余下像素点对应三维坐标
                }
                float tx = (u - u0) / fx;
                float ty = (v - v0) / fy;
                peo_plane[*pixels_nums].x = depth * tx/1000.0;
                peo_plane[*pixels_nums].y = -depth * ty/1000.0;
                peo_plane[*pixels_nums].z = -depth/1000.0;
                pixels[*pixels_nums].x = u;
                pixels[*pixels_nums].y = v;
                *pixels_nums = *pixels_nums+1;
            }
        }

        std::vector<double> plane_coeff;
        //最小二乘拟合一个平面
        if(*pixels_nums < 3){
            peo_direction = INFINITY;
            return peo_direction;
        }
        else{
            plane_coeff = humandetect::FittingPlaneZ(peo_plane,*pixels_nums);
        }
        //由平面法向量([plane_coeff[0], plane_coeff[1], plane_coeff[2]])，算出人体朝向
        float temp_theta = atan2(plane_coeff[0], plane_coeff[2]);
        peo_direction = atan2(sin(temp_theta), cos(temp_theta));
    }

    //无论人体前后，只计算人体这一平面法线朝着机器人这一侧的方向角，那么peo_direction一定是[-pi,-pi/2)或(pi/2,pi]之间的数
    if(peo_direction!=INFINITY){
        if(peo_direction>=-M_PI/2 && peo_direction<0){
            peo_direction = peo_direction + M_PI;
        }
        if(peo_direction>=0 && peo_direction<=M_PI/2){
            peo_direction = peo_direction - M_PI;
        }
    }
    return peo_direction;
};


/* 等待一致的人体朝向计算，主要是腿、髋、肩三对点都能计算出来、且朝向一致，视为此刻朝向计算正确
    本用于计算出初始正确朝向，后续以该朝向作为真实朝向抑制朝向的不鲁棒变化，但实际运行中，一旦出现一次朝向获取失败，
    下一帧又得等到重新获取正确朝向的那帧，在此期间只能暂停，十分影响跟随效果
*/
bool humandetect::wait_consistent_direction(std::vector<cv::Point3f> joints_coord,float * consistent_direction){
    float deepthreshold = 0.5;
    cv::Point3f invalid_point;invalid_point.x = 0;invalid_point.y = 0;invalid_point.z = 0;
    if(joints_coord[1]!=invalid_point && joints_coord[4]!=invalid_point && joints_coord[2]!=invalid_point && joints_coord[3]!=invalid_point && joints_coord[12]!=invalid_point && joints_coord[13]!=invalid_point){
        if(fabs(joints_coord[1].z-joints_coord[4].z)<deepthreshold && fabs(joints_coord[2].z-joints_coord[3].z)<deepthreshold && fabs(joints_coord[12].z-joints_coord[13].z)<deepthreshold){
            float theta32 = M_PI/2 + atan2(-joints_coord[2].x + joints_coord[3].x, -joints_coord[2].z + joints_coord[3].z);
            theta32 = atan2(sin(theta32), cos(theta32));
            float theta41 = M_PI/2 + atan2(-joints_coord[1].x + joints_coord[4].x, -joints_coord[1].z + joints_coord[4].z);
            theta41 = atan2(sin(theta41), cos(theta41));
            float theta1312 = M_PI/2 + atan2(-joints_coord[12].x + joints_coord[13].x, -joints_coord[12].z + joints_coord[13].z);
            theta1312 = atan2(sin(theta1312), cos(theta1312));
            if((theta32>0&&theta41>0&&theta1312>0)||(theta32<0&&theta41<0&&theta1312<0)){//朝向相同
                *consistent_direction = theta32;
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
};