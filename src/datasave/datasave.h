#ifndef DATASAVE_H_
#define DATASAVE_H_

#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "../humandetect/include/constant.hpp"


struct jointsinfo{
    double time;
    std::vector<cv::Point3f> joints;

    jointsinfo() = default;   // 添加默认构造函数

    jointsinfo(double time_init,std::vector<cv::Point3f> joints_init):time(time_init),joints(joints_init){}
};

struct obsinfo{
    int frameid;
    double frametime;
    std::vector<std::vector<float>> obs_position;//障碍物位置
    std::vector<int> frameids;//用于补偿的关键帧id
    std::vector<float> peo;//人体位置
    float yaw_origin;
    float yaw_current;

    obsinfo() = default;   // 添加默认构造函数

    obsinfo(int frameid_init,double frametime_init,std::vector<int> frameids_init,float (*obs_position_init)[2],float obs_nums_init,float * peo_init,float yaw_origin_init,float yaw_current_init):
    frameid(frameid_init),
    frametime(frametime_init),
    frameids(frameids_init),
    peo(3),
    yaw_origin(yaw_origin_init),
    yaw_current(yaw_current_init),
    obs_position(obs_nums_init,std::vector<float>(2))
    {
        peo[0] = peo_init[0];peo[1] = peo_init[1];peo[2] = peo_init[2];
        for(int i=0; i<obs_nums_init; i++){
            obs_position[i] = std::vector<float>{obs_position_init[i][0], obs_position_init[i][1]};
        }
    }
};


class DataSave{
    private:
        std::fstream _fjoints;//
        std::fstream _fobs;//
        std::vector<jointsinfo> _alljoints;
        std::vector<obsinfo> _allobs;

        std::vector<cv::Mat> _allrgb_forward;
        std::vector<cv::Mat> _allrgb_back;
        std::vector<cv::Mat> _allmap;

    public:
        DataSave(){
            _fjoints.open(JOINTS_DATA_PATH, std::ios::out | std::ios::trunc);
            _fobs.open(OBS_PATH, std::ios::out | std::ios::trunc);
        }

        ~DataSave(){
            _fjoints.close();
            _fobs.close();
        }

        void addjoints(std::vector<cv::Point3f> &joints, double t){
            jointsinfo temp(t,joints);
            _alljoints.push_back(temp);
        }

        void addobs(int frameid,double frametime,std::vector<int> frameids,float (*obs_position)[2],float obs_nums,float * peo,float yaw_origin,float yaw_current){
            obsinfo temp(frameid,frametime,frameids,obs_position,obs_nums,peo,yaw_origin,yaw_current);
            _allobs.push_back(temp);
        }

        void addrgb_forward(cv::Mat &framergb){
            cv::Mat temp;
            framergb.copyTo(temp);
            _allrgb_forward.push_back(temp);
        }

        void addrgb_back(cv::Mat &framergb){
            cv::Mat temp;
            framergb.copyTo(temp);
            _allrgb_back.push_back(temp);
        }

        void addmap(cv::Mat &framergb){
            cv::Mat temp;
            framergb.copyTo(temp);
            _allmap.push_back(temp);
        }

        bool savejoints();

        bool saveobs();

        bool savergb_forward();

        bool savergb_back();

        bool savemap();

};


class DebugSave{
    private:
        std::fstream _fdebuginfo;//
        std::vector<int> cycle_times;

    public:
        DebugSave(){
            _fdebuginfo.open(DEBUG_INFO_PATH, std::ios::out | std::ios::trunc);
        }

        ~DebugSave(){
            _fdebuginfo.close();
        }

        bool addcycle_times(int cycle_times_adding){
            cycle_times.push_back(cycle_times_adding);
        }


        bool savedebuginfo();

};

#endif
