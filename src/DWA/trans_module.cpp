#include "trans_module.h"

bool obs_trans(float (*obs_position)[2], int obs_nums, trans_param param2world){
    if(obs_nums == 0){
        return false;
    }
    else{
        for(int i = 0;i < obs_nums;i ++){
            obs_position[i][0] = obs_position[i][0]*cos(param2world.Theta) - obs_position[i][1]*sin(param2world.Theta) + param2world.TranslationX;
            obs_position[i][1] = obs_position[i][0]*sin(param2world.Theta) + obs_position[i][1]*cos(param2world.Theta) + param2world.TranslationY;
        }
        return true;
    }
}