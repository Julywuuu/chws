#ifndef TRANS_MODULE_H_
#define TRANS_MODULE_H_

#include <iostream>
#include "math.h"

struct trans_param
{
    float Theta;//逆时针旋转角度
    float TranslationX;//back or forward坐标系原点在world坐标系中的横坐标
    float TranslationY;//back or forward坐标系原点在world坐标系中的纵坐标
};

bool obs_trans(float (*obs_position)[2], int obs_nums, trans_param param2world);

#endif