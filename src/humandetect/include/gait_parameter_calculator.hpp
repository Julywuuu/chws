#pragma once
#include <cstdlib>
#include <vector>
#include <cstring>
#include <iomanip>
#include <math.h>
#include <sstream>
#include <fstream>

#include "constant.hpp"

#define NONE -1
#define IS_NONE(x) (x <= 0)

struct GaitDataUnit
{
    double t;
    std::vector<float> pos;

    GaitDataUnit(): t(0), pos(0) {}
    void init() {t=0; pos.clear();}
};


class GaitParaMeterCalculator
{
public:
    GaitParaMeterCalculator(double a);
    ~GaitParaMeterCalculator();
    std::string to_string(double t);
    bool cal(double t, std::vector<float> state, std::vector<float> orient, int gait);
private:
    int gait_cur;
    int gait_pre;
    double alpha;

    double gait_periods[4][2];  // 各步相持续时间为gait_periods[:, 1] - gait_periods[:, 0]

    GaitDataUnit gait_left_start;
    GaitDataUnit gait_right_start;
    GaitDataUnit gait_left_end;
    GaitDataUnit gait_right_end;

    GaitDataUnit left_start;
    double left_start_t;
    GaitDataUnit left_end;
    GaitDataUnit right_start;
    double right_start_t;
    GaitDataUnit right_end;

    double gait_cycle;
    double rate_gait_phase[4];
    double step_length;
    double step_width;
    double left_step_time;
    double right_step_time;
    double left_step_length;
    double right_step_length;

    // 文件
    std::ofstream f;
};