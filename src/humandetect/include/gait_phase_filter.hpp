#pragma once
#include "hidden_markov_model.hpp"
#include <iostream>
#include "constant.hpp"

class GaitPhaseFilter
{
public:
    GaitPhaseFilter(MatrixXf a, MatrixXf b);
    ~GaitPhaseFilter();
    void set(MatrixXf &a, MatrixXf &b);
    void init(int measure);
    void logical_filter(int estimation);
    void cal(int measure, bool valid);

    HMMFilter filter;
    int gait_cur;
    
private:
    int stop_time;
    int move_time;
    int gait_pre;
};