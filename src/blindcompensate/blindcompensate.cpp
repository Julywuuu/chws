#include "blindcompensate.h"
using namespace std;


//利用关键帧_kfsets进行障碍物数据的融合
vector<vector<float>> BlindCompensate::obsfusion(const keyframe currentframe){
    // int lastkeyframeid = _kfsets.lastaddframe();//获取当前最新的关键帧id
    vector<vector<float>> fuse_obs_position = currentframe.obs_position;
    for(int i = _kfsets.gethead(); i != _kfsets.gettail(); i = (i + 1) % _kfsets.framesets.size()){//遍历其他关键帧，进行数据融合
        keyframe frame = _kfsets.framesets[i]; // 按添加的先后顺序处理每个元素
        if(frame.frameid == currentframe.frameid){
            continue;
        }
        vector<vector<float>> obs_position = frame.obs_position;
        vector<float> peo = frame.peo;
        vector<float> relative_pos = get_transform(frame.robo_loc,currentframe.robo_loc);
        float sin_theta = sin(relative_pos[2]);float cos_theta = cos(relative_pos[2]);
        vector<float> obs_current(2);
        for(int j = 0; j < obs_position.size(); j++){
            //对于每一帧，忽略人体位置附近0.3m的障碍物
            if(obs_in_human(obs_position[j], peo)){
                continue;
            }
            //利用该帧和最新帧的相对位置关系，计算该帧障碍物信息在最新帧的位置
            obs_current[0] = obs_position[j][0] * cos_theta - obs_position[j][1] * sin_theta + relative_pos[0];
            obs_current[1] = obs_position[j][0] * sin_theta + obs_position[j][1] * cos_theta + relative_pos[1];
            //判断该位置是否当前帧的盲区，是则保存，否则忽略
            if(obs_in_blind(obs_current)){
                fuse_obs_position.push_back(obs_current);
            }
        }
    }
    //得到融合后的障碍物数据
    return fuse_obs_position;
};


//判断障碍物位置是否在盲区,在盲区的就要添加到当前帧中用于补偿
bool BlindCompensate::obs_in_blind(const vector<float> &obs){
    float obsx = obs[0];float obsy = obs[1];
    if(obsy > valid_ymax || obsy < valid_ymin ||
       obsx > valid_xmax || obsx < valid_xmin){
        return false;//不在有效地图范围
    }
    // else if(sqrt(pow(obsx,2)+pow(obsy,2)) < robotmodel_radius){
    //     return false;//由于误差投影到了小车模型范围内，显然盲区中不可能存在这样的障碍物
    // }
    else if(obsy < 0.25 && obsy > -0.25 &&//后向盲区已经小于小车碰撞检测半径，肯定不会碰撞了；而前向由于一直假设小车是后退的，所以其前面没有障碍物
            obsx < forwardblind_y && obsx > backblind_y){//所以前后小车宽度内的盲区设为无障碍物
        return false;
    }
    else{
        /*完整策略*/
        if(obsx > forwardblind_y){//前向障碍物
            float abs_y = fabs(obsx*tan(_forwardcam[1]/2.0));
            if(obsy <= abs_y && obsy >= -abs_y){
                return false;//位于可视范围
            }
            else{
                return true;
            }
        }

        // /*只看后向障碍物的策略*/
        // if(obsx > 0){//投影到了前向障碍物范围
        //     return false;
        // }


        else if(obsx < backblind_y){//后向障碍物
            float abs_y = fabs(obsx*tan(_backcam[1]/2.0));
            if(obsy <= abs_y && obsy >= -abs_y){
                return false;//位于可视范围
            }
            else{
                return true;
            }
        }
        else{//位于垂直视场角形成的盲区+对应水平盲区内
            return true;
        }
    }
};


//判断障碍物位置是否在人体位置附近
bool BlindCompensate::obs_in_human(const vector<float> &obs,const vector<float> &peo){
    float dist = std::sqrt(std::pow(obs[0] - peo[0], 2) + std::pow(obs[1] - peo[1], 2));
    return dist <= human_radius;    
};


// 输入世界坐标系下坐标系1和坐标系2的横纵坐标和x轴方向（该方向用弧度制表示）
//输出坐标系2中坐标系1的原点位置和x轴方向
vector<float> BlindCompensate::get_transform(const vector<float> &robo_loc1,const vector<float> &robo_loc2){
    vector<float> relative_pos(3);
    float dx = robo_loc1[0] - robo_loc2[0];
    float dy = robo_loc1[1] - robo_loc2[1];
    float theta = atan2(sin(atan2(dy, dx)-robo_loc2[2]),cos(atan2(dy, dx)-robo_loc2[2]));
    float d = sqrt(dx * dx + dy * dy);
    relative_pos[0] = d * cos(theta);
    relative_pos[1] = d * sin(theta);
    relative_pos[2] = robo_loc1[2] - robo_loc2[2];//顺时针旋转角度
    return relative_pos;    
};


//保存障碍物信息，用于调试----包括帧id，障碍物位置
bool BlindCompensate::obs_write(const int frameid, const vector<vector<float>> &obs){
    if (_fobs.is_open())
    {
        _fobs << "关键帧frameid = " <<frameid <<"\n";
        _fobs << std::fixed << std::setprecision(3);
        for (int i = 0; i < obs.size(); i++)
        {
            _fobs << obs[i][0] << "\t" << obs[i][1] << "\n";;
        }
        return true;
    }
    else
    {
        return false;
    }
};

        
