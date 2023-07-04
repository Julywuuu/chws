#include "processply/processply.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processply/processply.cpp"
#include "serialsend/serialsend.h"
#include "DWA/dwa_module.h"
#include <fstream>


using namespace obstacle_detection;
using namespace path_planning;
using namespace cv;
using namespace std;
using namespace serial;

typedef float Position[2];

int * get_target_pixel_coord(cv::Mat color){
    Mat hsvImage;
    cvtColor(color,hsvImage,COLOR_BGR2HSV);

    // 颜色阈值化处理
    cv::Mat TargetMat;
    cv::inRange(hsvImage, cv::Scalar(45, 60, 60),cv::Scalar(55, 255, 255), TargetMat);

	int * target_pixel_coord = new int[2];
    target_pixel_coord[0] = 0;
    target_pixel_coord[1] = 0;
    int target_nums = 0;
    for(int row = 0; row <TargetMat.rows; row++){
        uchar *ptr = TargetMat.data + row*TargetMat.cols;
        for(int col = 0; col < color.cols; col++){
            if((int)ptr[col] == 255){
                target_pixel_coord[0] += col;
                target_pixel_coord[1] += row;
                target_nums++;
            }
        }
    }
    cv::imshow("TargetMat", TargetMat);

    //求平均，取出目标点的像素坐标
    target_pixel_coord[0] = (int)(target_pixel_coord[0]/target_nums);
    target_pixel_coord[1] = (int)(target_pixel_coord[1]/target_nums);

    return target_pixel_coord;

}

bool write_error(ofstream &outfile, int cycle_times, float camera_direction_error, float robot_direction_error, float dist_to_peo){
    if (!outfile)
    {
        cout << "打开文件失败" << endl;
        return false;
    }
    //向文件中写入数据
    outfile << cycle_times <<"          "<< 
    camera_direction_error <<"          "<<
    robot_direction_error  <<"          "<<
    dist_to_peo<< endl;
    return true;
}


int main(int argc, char *argv[]) {
    std::cout << "starting enviroment" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudI;
    int cycle_times = 1;

//实时采集
    ProcessPointClouds<pcl::PointXYZ> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZ>();
    rs2::pipeline p = pointProcessorI->startDepthCamera();//初始化图像采集对象
    serialsend ser("/dev/ttyTHS1",115200);//初始化串口对象
    DWA dwaplanning;//初始化路径规划对象
    robo_state x = {0,0,0,0,0,0};//初始化机器人状态

    ofstream outfile;    //定义输出流对象
    outfile.open("../src/sensors/errordata/errordata.txt");    //打开文件

    while(1){
        std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, Mat> PCL_Mat = pointProcessorI->Points2PCL_Mat(p);
        inputCloudI = PCL_Mat.first;
        Mat color = PCL_Mat.second;

        int *target_pixel_coord = get_target_pixel_coord(color);
        float * coord = pointProcessorI->get_3d_camera_coordinate(inputCloudI,target_pixel_coord[0],target_pixel_coord[1]);

        float peo[3] = {-coord[2],-coord[0],INFINITY};

        float speed[3] = {9,9,9};
        ser.send(speed);

        //可视化，便于调试
        std::cout <<"target_pixel = [  "<<target_pixel_coord[0]<< " , "<<target_pixel_coord[1]<<"  ]"<<std::endl;
        std::cout <<"target_coord = [  "<<coord[0]<< " , "<<coord[1]<<" , "<<coord[2]<<"  ]"<<std::endl;
        cv::circle(color, Point(target_pixel_coord[0], target_pixel_coord[1]), 3, Scalar(0, 0, 255),3);
        cv::imshow("color", color);
        
        stringstream ss;
        //调试信息--保存运行过程中的捕获目标图像
        ss << setw(6) << setfill('0') << cycle_times ;
        String target_file = "../src/sensors/target/" + ss.str() + ".png";
        cv::imwrite(target_file, color);

        //调试信息--保存每一帧图像对应相机朝向偏差、距离偏差、(机器人相对人体方向偏差)
        float camera_direction_error = dwaplanning.calc_camera_direction_error(peo,x);
        float robot_direction_error = dwaplanning.calc_robot_direction_error(peo,x);
        float dist_to_peo = sqrt(pow(x[0]-peo[0],2)+pow(x[1]-peo[1],2));
        if(target_pixel_coord[0]==0&&target_pixel_coord[1]==0){
            std::cout<<"第"<<cycle_times<<"帧的"<<"视野中没有目标..."<<std::endl;
            camera_direction_error = INFINITY;
            robot_direction_error = INFINITY;
            dist_to_peo = INFINITY;
        }
        //store cycle_times , theta and dist_to_peo with *.txt
        write_error(outfile,cycle_times,camera_direction_error,robot_direction_error,dist_to_peo);
        std::cout <<"cycle_times =  "<<cycle_times<<std::endl;
        std::cout <<"camera_direction_error =  "<<camera_direction_error/M_PI*180<<"°"<<std::endl;
        std::cout <<"robot_direction_error =  "<<robot_direction_error/M_PI*180<<"°"<<std::endl;
        std::cout <<"dist_to_peo =  "<<dist_to_peo<<"m"<<std::endl;
        
        
        delete []target_pixel_coord;
        delete []coord;
        cycle_times++;
        waitKey(1);
    }
    outfile.close();    //关闭文件
    return 0;
}
