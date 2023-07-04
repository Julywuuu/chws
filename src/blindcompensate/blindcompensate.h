#ifndef BLINDCOMPENSATE_H_
#define BLINDCOMPENSATE_H_

#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include "../humandetect/include/constant.hpp"
#include "../DWA/dwa_module.h"


struct keyframe{//定义为const报错
     int frameid;
     double frametime;
     std::vector<float> robo_loc;//对应机器人姿态
     std::vector<float> peo;//对应人体姿态，用于去除动态人体的影响
     std::vector<std::vector<float>> obs_position;//障碍物位置

     keyframe() = default;   // 添加默认构造函数

     keyframe(int frameid_init,double frametime_init,float * robo_loc_init,float * peo_init,float (*obs_position_init)[2],float obs_nums_init):
     frameid(frameid_init),
     frametime(frametime_init),
     robo_loc(robo_loc_init,robo_loc_init+3),
     peo(peo_init,peo_init+3),
     obs_position(obs_nums_init,std::vector<float>(2))
     {
        for(int i=0; i<obs_nums_init; i++){
            obs_position[i] = std::vector<float>{obs_position_init[i][0], obs_position_init[i][1]};
        }
     }
};


class KeyFrameSets{
    private:
        int _head;
        int _tail;

    public:
        std::vector<keyframe> framesets;

        KeyFrameSets(int framenums) : framesets(framenums+1), _head(0), _tail(0) {}//由于tail指向的元素是不能用的，所以队列实际的容量为framenums

        void addframe(keyframe frame) {
            framesets[_tail] = frame;
            _tail = (_tail + 1) % framesets.size();
            if (_tail == _head) {
                _head = (_head + 1) % framesets.size();
            }
        }

        int lastaddframe(){
            int frameid;
            if(this->is_empty()){
                std::cout<<"队列为空,上一添加的关键帧不存在"<<std::endl;
                return -1;
            }
            else{
                if(_tail == 0){
                    frameid = framesets.size()-1;
                }
                else{
                    frameid = _tail-1;
                }
                return frameid;
            }
        }

        keyframe remove() {
            keyframe frame = framesets[_head];
            _head = (_head + 1) % framesets.size();
            return frame;
        }

        int gethead(){
            return _head;
        }

        int gettail(){
            return _tail;
        }

        int is_empty(){
            return _head == _tail; // 头索引和尾索引相等，那么该队列为空
        }

        int is_full(){
            return (_tail+1)%framesets.size() == _head; // 若尾索引+1的值 取模 SIZE 等于 头索引，则队列已满
        }


};

class BlindCompensate{
    private:
        KeyFrameSets _kfsets;//存储关键帧
        std::fstream _fobs;//
        std::vector<float> _forwardcam;//存储前面相机的高度，水平视场角，垂直视场角
        std::vector<float> _backcam;
        float _obs_scan_height;
        float valid_ymax = 1.5;//地图有效范围
        float valid_ymin = -1.5;
        float valid_xmax = 3.0;
        float valid_xmin = -3.0;
        // path_planning::Config cfg;
        // float robotmodel_radius = cfg.robot_radius;//小车模型范围    
        float robotmodel_radius = 0.4;//待小车障碍物补偿机制健全后可设为小车模型大小0.35
        float human_radius = 0.3;//人体位置附近的判定距离
        float forwardblind_y;//垂直视场角形成的盲区
        float backblind_y;

    public:
        BlindCompensate(int framenums,std::vector<float> forwardcam,std::vector<float> backcam, float obs_scan_height) :
         _kfsets(framenums),
         _forwardcam(forwardcam),
         _backcam(backcam),
         _obs_scan_height(obs_scan_height)
        {
            _fobs.open(OBS_PATH, std::ios::out | std::ios::trunc);
            //减_obs_scan_height得到障碍物所在平面的盲区，减0.02使盲区判定更严格，避免相机波动获取的障碍物经盲区补偿后投射到当前帧盲区边缘，被判定有效
            forwardblind_y = (_forwardcam[0]-_obs_scan_height-0.02)/tan(_forwardcam[2]/2.0);
            // backblind_y = -(_backcam[0]-_obs_scan_height-0.02)/tan(_backcam[2]/2.0);
            backblind_y = -(_backcam[0]-_obs_scan_height+0.09)/tan(_backcam[2]/2.0);//使得backblind_y=-0.5左右，增大后向盲区范围用于补偿
        }

        ~BlindCompensate(){_fobs.close();};

        void addframe(keyframe frame){
            _kfsets.addframe(frame);
        }

        //用于主函数中，判断是否是关键帧
        int last_robo_loc(std::vector<float> &lastroboloc){
            int lastkeyframeid = _kfsets.lastaddframe();
            if(lastkeyframeid == -1){
                std::cout<<"上一关键帧机器人位置不存在"<<std::endl;
                return 0;
            }
            else{
                lastroboloc = _kfsets.framesets[lastkeyframeid].robo_loc;
                return 1;
            }
        }

        //用于主函数中，判断返回队列中所有关键帧的id
        std::vector<int> get_frameids(){
            std::vector<int> frameids(_kfsets.framesets.size(),-1);
            for(int i = _kfsets.gethead(),j=0; i != _kfsets.gettail(); i = (i + 1) % _kfsets.framesets.size(),j++){
                frameids[j] = _kfsets.framesets[i].frameid;
            }
            return frameids;
        }

        //利用关键帧_kfsets进行障碍物数据的融合
        std::vector<std::vector<float>> obsfusion(const keyframe currentframe);

        //判断障碍物位置是否在盲区
        bool obs_in_blind(const std::vector<float> &obs);

        //判断障碍物位置是否在人体位置附近
        bool obs_in_human(const std::vector<float> &obs,const std::vector<float> &peo);

        //获取相对位置关系
        std::vector<float> get_transform(const std::vector<float> &robo_loc1,const std::vector<float> &robo_loc2);

        //保存障碍物信息，用于调试----包括帧id，障碍物位置
        bool obs_write(const int frameid, const std::vector<std::vector<float>> &obs);

};

#endif
