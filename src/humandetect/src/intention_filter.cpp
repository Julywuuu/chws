#include "intention_filter.hpp"

IntentionFilter::IntentionFilter(MatrixXf a, MatrixXf b):
    filter(a, b)
{
    intention = intention_mode["stop"];
    stop_time = 0;
    move_time = 0;
}

IntentionFilter::~IntentionFilter()
{

}

void IntentionFilter::set(MatrixXf &a, MatrixXf &b)
{
    filter.set(a, b);
}

void IntentionFilter::init(int measure)
{
    filter.init(measure);
}

void IntentionFilter::logical_filter(int estimation)
{
    if (intention == intention_mode["stop"])
    {
        if (stop_time <= 7)
        {
            stop_time++;
        }
        else
        {
            move_time = 0;
            intention = estimation;
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
            intention = estimation;
        }
    }
}

void IntentionFilter::cal(int measure, bool valid)
{
    std::cout << "measure:" << measure << std::endl;
    filter.cal(measure, valid);
    int estimation = 0;
    float maxval = 0;
    for (int i = 0; i < filter.state.rows(); i++)
    {
        std::cout << filter.state(i, 0) << "\t";
        if (filter.state(i, 0) > maxval)
        {
            maxval = filter.state(i, 0);
            estimation = i;
        }
    }
    std::cout << std::endl;
    logical_filter(estimation);
    std::cout << "intention:" << intention << std::endl;
}