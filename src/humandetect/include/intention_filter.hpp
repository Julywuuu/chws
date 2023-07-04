#pragma once
#include "hidden_markov_model.hpp"
#include "constant.hpp"
#include <opencv2/opencv.hpp>

class IntentionFilter
{
public:
    IntentionFilter(MatrixXf a, MatrixXf b);
    ~IntentionFilter();
    void set(MatrixXf &a, MatrixXf &b);
    void init(int measure);
    void logical_filter(int estimation);
    void cal(int measure, bool valid);

    HMMFilter filter;
    int intention;

private:
    int stop_time;
    int move_time;
};