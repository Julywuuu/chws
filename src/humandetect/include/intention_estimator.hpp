#pragma once
#include <math.h>
#include <opencv2/opencv.hpp>

#include "constant.hpp"
#include "kalman_filter.hpp"
#include "intention_filter.hpp"
#include "gait_phase_filter.hpp"
#include "gait_parameter_calculator.hpp"

// using namespace kalman_filter;

class IntentionEstimator
{
public:
    IntentionEstimator(double dt, std::vector<float> q, std::vector<float> r);
    ~IntentionEstimator();
    float get_orient();
    std::vector<float> get_linear_velocity();
    float get_angle_velocity();

    float get_human_direction();

    int get_intention();
    int get_gait_phase(double t);
    int cal(cv::Point3f j0, cv::Point3f j2, cv::Point3f j3, cv::Point3f j5);


    kalman_filter::KalmanFilter state_filter;
    kalman_filter::KalmanFilter orient_filter;
    IntentionFilter intention_filter;
    GaitPhaseFilter gaitphase_filter;
    GaitParaMeterCalculator gaitparam_calculator;
    float thres_valid;
    float thres_cv;
    float thres_lv;
    float thres_w;
    float thres_diff;
    int frame_loss;
    int frame_flip;
    std::vector<float> loc_pre;
    bool det_missing;
    
private:
    bool intention_done;
};