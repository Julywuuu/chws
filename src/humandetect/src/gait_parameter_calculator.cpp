#include <gait_parameter_calculator.hpp>

GaitParaMeterCalculator::GaitParaMeterCalculator(double a=0.8):
    gait_periods{{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    rate_gait_phase{NONE, NONE, NONE, NONE}
{
    gait_cur = 0;
    gait_pre = 0;
    alpha = a;

    gait_cycle = NONE;
    step_length = NONE;
    step_width = NONE;
    left_step_time = NONE;
    right_step_time = NONE;
    left_step_length = NONE;
    right_step_length = NONE;

    // 打开文件
    f.open(GAIT_PARAMETER_PATH, std::ios::out | std::ios::trunc);
}

GaitParaMeterCalculator::~GaitParaMeterCalculator()
{
    // 关闭文件
    f.close();
}

std::string GaitParaMeterCalculator::to_string(double t)
{
    std::stringstream stream;
    stream << "time: " << std::fixed << std::setprecision(3) << t << std::endl;
    stream << "phase: " << gait_phase_inv[gait_cur] << std::endl;
    stream << "gait_cycle: " << gait_cycle << std::endl;
    stream << "rate_gait_phase: [" << rate_gait_phase[0] << ", "
                                   << rate_gait_phase[1] << ", " 
                                   << rate_gait_phase[2] << ", "
                                   << rate_gait_phase[3] << "]"
                                   << std::endl;
    stream << "step_length: " << step_length << std::endl;
    stream << "step_width: " << step_width << std::endl;
    stream << "left_step_time: " << left_step_time << std::endl;
    stream << "right_step_time: " << right_step_time << std::endl;
    stream << "left_step_length: " << left_step_length << std::endl;
    stream << "right_step_length: " << right_step_length << std::endl;
    stream << "---------------------\n";
    return stream.str();
}

bool GaitParaMeterCalculator::cal(double t, std::vector<float> state, std::vector<float> orient, int gait)
{
    float dt = 0.01;  // 时间间隔阈值
    gait_cur = gait;

    if (gait == gait_phase["stop"])
    {
        memset(gait_periods, 0, 8*sizeof(float));
        gait_left_start.init();
        gait_right_start.init();
        left_start.init();
        left_end.init();
        right_start.init();
        right_end.init();
    }

    if (gait_cur != gait_phase["stop"] && gait_pre != gait_phase["stop"] && gait_cur != gait_pre)
    {
        gait_periods[(gait_cur - gait_phase["left_swing"] + 4 - 1) % 4][1] = t;
    }

    if (gait_pre == gait_phase["left_swing"] && gait_cur == gait_phase["left_support"])
    {
        left_start = left_end;
        if (left_start.pos.size() > 0)
            left_start.t = left_start_t;
        left_end.t = t;
        left_end.pos = std::vector<float>{state[s["lx"]], state[s["ly"]], state[s["lz"]]};

        right_start_t = t;
        
        gait_left_start = gait_left_end;
        gait_left_end.t = t;
        gait_left_end.pos = std::vector<float>{
            (state[s["lx"]] + state[s["rx"]]) / 2,
            (state[s["ly"]] + state[s["ry"]]) / 2,
            (state[s["lz"]] + state[s["rz"]]) / 2
        };
    }

    if (gait_pre == gait_phase["right_swing"] && gait_cur == gait_phase["right_support"])
    {
        left_start_t = t;

        right_start = right_end;
        if (right_start.pos.size() > 0)
            right_start.t = right_start_t;
        right_end.t = t;
        right_end.pos = std::vector<float>{state[s["rx"]], state[s["ry"]], state[s["rz"]]};

        gait_right_start = gait_right_end;
        gait_right_end.t = t;
        gait_right_end.pos = std::vector<float>{
            (state[s["lx"]] + state[s["rx"]]) / 2,
            (state[s["ly"]] + state[s["ry"]]) / 2,
            (state[s["lz"]] + state[s["rz"]]) / 2
        };
    }

    float length, period;
    // 计算左侧单侧参数
    if (left_start.pos.size() > 0 && left_end.pos.size() > 0)
    {
        if (left_end.t - left_start.t > dt)
        {
            period = left_end.t - left_start.t;
            left_step_time = IS_NONE(left_step_time) ? period 
                : (alpha * left_step_time + (1-alpha) * period);
            length = std::sqrt(
                std::pow(left_end.pos[0] - left_start.pos[0], 2) 
                + std::pow(left_end.pos[1] - left_start.pos[1], 2) 
                + std::pow(left_end.pos[2] - left_start.pos[2], 2)
            );
            left_step_length = IS_NONE(left_step_length) ? length 
                : (alpha * left_step_length + (1-alpha) * length);
        }
    }
    // 计算右侧单侧参数
    if (right_start.pos.size() > 0 && right_end.pos.size() > 0)
    {
        if (right_end.t - right_start.t > dt)
        {
            period = right_end.t - right_start.t;
            right_step_time = IS_NONE(right_step_time) ? period 
                : (alpha * right_step_time + (1-alpha) * period);
            length = std::sqrt(
                std::pow(right_end.pos[0] - right_start.pos[0], 2) 
                + std::pow(right_end.pos[1] - right_start.pos[1], 2) 
                + std::pow(right_end.pos[2] - right_start.pos[2], 2)
            );
            right_step_length = IS_NONE(right_step_length) ? length 
                : (alpha * right_step_length + (1-alpha) * length);
        }
    }
    // 计算步态周期、步长
    if (gait_left_start.pos.size() > 0 && gait_left_end.pos.size() > 0)
    {
        if (gait_right_start.pos.size() > 0 && gait_right_end.pos.size() > 0)
        {
            if (gait_left_end.t - gait_left_start.t > dt && gait_right_end.t - gait_right_start.t > dt)
            {
                period = (gait_left_end.t + gait_right_end.t - gait_left_start.t - gait_right_start.t) / 2;
                gait_cycle = IS_NONE(gait_cycle) ? (period)
                    : (alpha * gait_cycle + (1-alpha) * period);
                length = std::sqrt(
                    std::pow(gait_left_end.pos[0] - gait_left_start.pos[0], 2) 
                    + std::pow(gait_left_end.pos[1] - gait_left_start.pos[1], 2) 
                    + std::pow(gait_left_end.pos[2] - gait_left_start.pos[2], 2)
                );
                length = (length + std::sqrt(
                    std::pow(gait_right_end.pos[0] - gait_right_start.pos[0], 2) 
                    + std::pow(gait_right_end.pos[1] - gait_right_start.pos[1], 2) 
                    + std::pow(gait_right_end.pos[2] - gait_right_start.pos[2], 2)
                )) / 2;
                step_length = IS_NONE(step_length) ? length
                    : (alpha * step_length + (1-alpha) * length);
            }
        }
    }

    float width;
    // 计算步宽
    if (gait_cur == gait_phase["left_support"] || gait_cur == gait_phase["right_support"])
    {
        width = std::abs(
            std::cos(orient[0] - M_PI/2) * (state[s["ly"]] - state[s["ry"]])
            + std::sin(orient[0] - M_PI/2) * (state[s["lz"]] - state[s["rz"]])
        );
        step_width = IS_NONE(step_width) ? width
            : (alpha * step_width + (1-alpha) * width);
    }

    // 计算步相占比
    double periods[4] = {0, 0, 0, 0};
    double tot;
    bool flag;
    if (gait_cur != gait_phase["stop"] and gait_cur != gait_pre)
    {
        flag = true;
        for (int i = 0; i < 4; i++)
        {
            if (gait_periods[i][1] - gait_periods[i][0] > dt && !IS_NONE(gait_periods[i][0]))
                periods[i] = gait_periods[i][1] - gait_periods[i][0];
            else
                flag = false;
        }
        if (flag)
        {
            tot = periods[0] + periods[1] + periods[2] + periods[3];
            for (int i = 0; i < 4; i++)
            {
                periods[i] /= tot;
                rate_gait_phase[i] = IS_NONE(rate_gait_phase[i]) ? periods[i]
                    : (alpha * rate_gait_phase[i] + (1-alpha) * periods[i]);
            }

        }
    }

    // 统计完步相占比后再更新对应前一时刻值
    if (gait_cur != gait_phase["stop"] && gait_pre != gait_phase["stop"] && gait_cur != gait_pre)
    {
        gait_periods[gait_cur - gait_phase["left_swing"]][0] = t;
    }

    // 记录上一步相
    gait_pre = gait_cur;

    // 保存数据
    if(f.is_open())
    {
        f << to_string(t);
        return true;
    }
    else
    {
        return false;
    }
}
