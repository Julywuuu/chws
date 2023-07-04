#include "gait_phase_filter.hpp"

GaitPhaseFilter::GaitPhaseFilter(MatrixXf a, MatrixXf b):
    filter(a, b)
{
    gait_cur = gait_phase["stop"];
    gait_pre = gait_phase["stop"];
    stop_time = 0;
    move_time = 0;
}

GaitPhaseFilter::~GaitPhaseFilter()
{

}

void GaitPhaseFilter::set(MatrixXf &a, MatrixXf &b)
{
    filter.set(a, b);
}

void GaitPhaseFilter::init(int measure)
{
    filter.init(measure);
}

void GaitPhaseFilter::logical_filter(int estimation)
{
    if (estimation == gait_phase["stop"])
    {
        if (stop_time <= 7)
        {
            stop_time++;
        }
        else
        {
            move_time = 0;
            gait_cur = estimation;
        }
    }
    else
    {
        if (move_time <= 2)
        {
            move_time++;
        }
        else
        {
            stop_time = 0;
            if (gait_pre != gait_phase["stop"])
            {
                if ((estimation + 4 - gait_pre) % 4 <= 1)  // 如果步相变化小于1步
                    gait_cur = estimation;
                else if (gait_pre == gait_phase["left_swing"] && estimation == gait_phase["right_swing"])  // 如果跳过了左支撑相
                    gait_cur = gait_phase["left_support"];
                else if (gait_pre == gait_phase["right_swing"] && estimation == gait_phase["left_swing"])  // 如果跳过了右支撑相
                    gait_cur = gait_phase["right_support"];
            }
            else
            {
                gait_cur = estimation;
            }
        }
    }
    gait_pre = gait_cur;
}

void GaitPhaseFilter::cal(int measure, bool valid)
{
    std::cout << "gait measure:" << measure << std::endl;
    filter.cal(measure, valid);
    int estimation = 0;
    float maxval = 0;
    for (int i = 0; i < filter.state.rows(); i++)
    {
        if (filter.state(i, 0) > maxval)
        {
            maxval = filter.state(i, 0);
            estimation = i;
        }
    }
    logical_filter(estimation);
    std::cout << "gait_phase:" << gait_cur << std::endl;
}