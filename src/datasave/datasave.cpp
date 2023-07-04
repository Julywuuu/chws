#include "datasave.h"
using namespace std;


//保存关节点信息
bool DataSave::savejoints(){
    if(_fjoints.is_open()){
        for(int i = 0; i < _alljoints.size(); i++){
            _fjoints << std::fixed << std::setprecision(6) << _alljoints[i].time;
            _fjoints << std::fixed << std::setprecision(3);
            for (int j = 0; j < _alljoints[i].joints.size(); j++)
            {
                _fjoints << "\t" << _alljoints[i].joints[j].x << "\t" << _alljoints[i].joints[j].y << "\t" << _alljoints[i].joints[j].z;
            }
            _fjoints << "\n";
        }
        std::cout<<std::dec<<"关节点信息写入文件成功，总帧数 = "<<_alljoints.size()<<std::endl;
        return true;
    }
    else
    {
        std::cout<<"关节点信息写入文件失败"<<std::endl;
        return false;
    }
};


//保存障碍物信息
bool DataSave::saveobs(){
    if(_fobs.is_open()){
        for(int i = 0; i < _allobs.size(); i++){
            _fobs << "关键帧frameid = " <<_allobs[i].frameid <<"\n";
            _fobs << "小车原始朝向yaw_origin = "<<_allobs[i].yaw_origin <<"\n";
            _fobs << "小车当前朝向yaw_current = "<<_allobs[i].yaw_current <<"\n";
            for(int p = 0; p < _allobs[i].peo.size(); p++){
                _fobs << "人体姿态 = " <<_allobs[i].peo[p] <<"\n";
            }
            for(int k = 0; k < _allobs[i].frameids.size(); k++){
                _fobs << "用于补偿该帧的frameids = " <<_allobs[i].frameids[k] <<"\n";
            }
            _fobs << std::fixed << std::setprecision(3);
            for (int j = 0; j < _allobs[i].obs_position.size(); j++)
            {
                _fobs << _allobs[i].obs_position[j][0] << "\t" << _allobs[i].obs_position[j][1] << "\n";
            }
        }
        std::cout<<std::dec<<"障碍物信息写入文件成功，总帧数 = "<<_allobs.size()<<std::endl;
        return true;
    }
    else
    {
        std::cout<<"障碍物信息写入文件失败"<<std::endl;
        return false;
    }
};


//保存视频
bool DataSave::savergb_forward(){
	cv::VideoWriter video_writer;
    if(video_writer.open("/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/forward.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 16, cv::Size(640, 480))){
        for(int i = 0; i < _allrgb_forward.size(); i++){
            video_writer.write(_allrgb_forward[i]);
        }
        std::cout<<std::dec<<"forward视频信息保存成功，总帧数 = "<<_allrgb_forward.size()<<std::endl;
        video_writer.release();
        return true;
    }
    else
    {
        std::cout<<"forward视频信息保存失败"<<std::endl;
        return false;
    }
};

//保存视频
bool DataSave::savergb_back(){
	cv::VideoWriter video_writer;
    if(video_writer.open("/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/back.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 16, cv::Size(640, 480))){
        for(int i = 0; i < _allrgb_back.size(); i++){
            video_writer.write(_allrgb_back[i]);
        }
        std::cout<<std::dec<<"back视频信息保存成功，总帧数 = "<<_allrgb_back.size()<<std::endl;
        video_writer.release();
        return true;
    }
    else
    {
        std::cout<<"back视频信息保存失败"<<std::endl;
        return false;
    }
};

//保存地图
bool DataSave::savemap(){
	cv::VideoWriter video_writer;
    if(video_writer.open("/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/map.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 16, _allmap[0].size())){
        for(int i = 0; i < _allmap.size(); i++){
            video_writer.write(_allmap[i]);
        }
        std::cout<<std::dec<<"map地图信息保存成功，总帧数 = "<<_allmap.size()<<std::endl;
        video_writer.release();
        return true;
    }
    else
    {
        std::cout<<"map地图信息保存失败"<<std::endl;
        return false;
    }    
};



bool DebugSave::savedebuginfo(){
    if(_fdebuginfo.is_open()){

        for (int i = 0; i < cycle_times.size(); i++){
            _fdebuginfo << "cycle_times = " << cycle_times[i]<< "\n";
        }



        std::cout<<std::dec<<"调试信息写入文件成功"<<std::endl;
        return true;
    }
    else
    {
        std::cout<<"调试信息写入文件失败"<<std::endl;
        return false;
    }
};
