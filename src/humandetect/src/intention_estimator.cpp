#include "intention_estimator.hpp"

IntentionEstimator::IntentionEstimator(double dt, std::vector<float> q, std::vector<float> r):
    state_filter(8, dt, q, r),
    orient_filter(1, dt, std::vector<float>{20, 100}, std::vector<float>{100}),
    intention_filter(MatrixXf::Zero(4, 4), MatrixXf::Zero(4, 4)),
    gaitphase_filter(MatrixXf::Zero(5, 5), MatrixXf::Zero(4, 5)),
    gaitparam_calculator(0.8),
    loc_pre(0)
    // gaitparam_calculator
{
    MatrixXf Ai(4, 4), Bi(4, 4);
    Ai << 0.7, 0.06, 0.06, 0.06,
          0.1, 0.82, 0.06, 0.06,
          0.1, 0.06, 0.82, 0.06,
          0.1, 0.06, 0.06, 0.82;
    Bi << 0.88, 0.04, 0.04, 0.04,
          0.04, 0.88, 0.04, 0.04,
          0.04, 0.04, 0.88, 0.88,
          0.04, 0.04, 0.04, 0.04;
    intention_filter.set(Ai, Bi);
    // intention_filter.init(0);

    MatrixXf Ag(5, 5), Bg(4, 5);
    Ag << 0.6, 0.1, 0.1, 0.1, 0.1,
          0.2, 0.8, 0.0, 0.0, 0.1,
          0.0, 0.1, 0.8, 0.0, 0.0,
          0.2, 0.0, 0.1, 0.8, 0.0,
          0.0, 0.0, 0.0, 0.1, 0.8;
    Bg << 0.7, 0.05, 0.05, 0.05, 0.05,
          0.1, 0.85, 0.05, 0.05, 0.05,
          0.1, 0.05, 0.05, 0.85, 0.05,
          0.1, 0.05, 0.85, 0.05, 0.85;
    gaitphase_filter.set(Ag, Bg);
    // gaitphase_filter.init(0);


    // gaitparam_calculator

    thres_valid = 1;
    thres_cv = 20;
    thres_lv = 36;
    thres_w = M_PI / 20;
    thres_diff = 5 * M_PI / 12;
    frame_loss = 0;
    frame_flip = 0;
    det_missing = false;
    intention_done = false;
}

IntentionEstimator::~IntentionEstimator()
{

}

float IntentionEstimator::get_orient()
{
    int n2pi = std::round(std::fabs(orient_filter.state(0, 0) - M_PI) / 2 / M_PI);
    int sign = (M_PI - orient_filter.state(0, 0) > 0) ? 1 : ((M_PI - orient_filter.state(0, 0) < 0) ? -1 : 0);
    return orient_filter.state(0, 0) + sign * n2pi * 2 * M_PI;
}

std::vector<float> IntentionEstimator::get_linear_velocity()
{
    float theta = get_orient();
    float amp = std::sqrt(
        std::pow((state_filter.state(s["dly"], 0) + state_filter.state(s["dry"], 0)) / 2, 2) 
        + std::pow((state_filter.state(s["dlz"], 0) + state_filter.state(s["drz"], 0)) / 2, 2)
    );
    return std::vector<float>{amp * std::cos(theta), amp * std::sin(theta)};
}

float IntentionEstimator::get_angle_velocity()
{
    return orient_filter.state(1, 0);
}

float IntentionEstimator::get_human_direction()
{
    return state_filter.state(s["o"], 0);
}

int IntentionEstimator::get_intention()
{
    float theta = state_filter.state(s["o"], 0);
    float v = std::sqrt(
        std::pow((state_filter.state(s["dly"], 0) + state_filter.state(s["dry"], 0)) / 2, 2) 
        + std::pow((state_filter.state(s["dlz"], 0) + state_filter.state(s["drz"], 0)) / 2, 2)
    );
    float v_phi = std::atan2(
        state_filter.state(s["dlz"], 0) + state_filter.state(s["drz"], 0),
        state_filter.state(s["dly"], 0) + state_filter.state(s["dry"], 0)
    );
    float wv_phi = state_filter.state(s["dvo"], 0);
    float w = state_filter.state(s["do"], 0);

    int n2pi = std::round(std::abs(orient_filter.state(0, 0) - theta) / 2 / M_PI);
    int sign = (orient_filter.state(0, 0) - theta > 0) ? 1 : ((orient_filter.state(0, 0) - theta < 0) ? -1 : 0);
    theta += sign * n2pi * 2 * M_PI;

    n2pi = std::round(std::abs(orient_filter.state(0, 0) - v_phi) / 2 / M_PI);
    sign = (orient_filter.state(0, 0) - v_phi > 0) ? 1 : ((orient_filter.state(0, 0) - v_phi < 0) ? -1 : 0);
    v_phi += sign * n2pi * 2 * M_PI;

    bool v_move = v > thres_cv;                  // 位置是否明显变化
    bool wv_move = wv_phi > thres_w;             // 运动方向是否有明显变化
    bool w_move = std::abs(w) > thres_w;         // 朝向是否有明显变化
    float diff_v_o = std::abs(v_phi - theta);    // 朝向与运动方向的差异

    std::cout << "v=" << v << ", w=" << w << ", o=" << state_filter.state(s["o"], 0) << std::endl;
    std::cout << (~v_move && ~w_move) << std::endl;
    std::cout << (!v_move && !w_move) << std::endl;
    int ret = 0;
    float orient;
    // 没有明显运动
    if (!v_move && !w_move)
    {
        ret = intention_mode["stop"];
        orient = theta;  // 选择身体朝向为意图方向
    }
    // 朝向有明显改变
    else if (w_move)
    {
        ret = intention_mode["turn"];
        orient = theta;  // 选择身体朝向为意图方向
    }
    // 位置改变 & 运动方向改变有限 & 朝向与运动方向差异有限
    else if (v_move && !wv_move && diff_v_o < thres_diff)
    {
        ret = intention_mode["forward"];
        float sigma_theta = std::sqrt(state_filter.cov(s["o"], s["o"]));
        float sigma_v_phi = std::sqrt(state_filter.cov(s["vo"], s["vo"]));
        orient = (sigma_v_phi * theta + sigma_theta * v_phi) / (sigma_theta + sigma_v_phi);  // 选择朝向加权为意图方向
    }
    // 位置改变 & 运动方向改变有限 & 朝向与运动方向差异明显
    else if (v_move && !wv_move && diff_v_o >= thres_diff)
    {
        ret = intention_mode["lateral"];
        orient = v_phi;  // 选择速度朝向为意图方向
    }
    // 速度的方向明显改变
    else if (wv_move)
    {
        ret = intention_mode["turn"];
        orient = theta;  // 选择身体朝向为意图方向
    }

    intention_filter.cal(ret, true);
    orient_filter.cal(std::vector<float>(1, orient), true);

    intention_done = true;

    return intention_filter.intention;
}

int IntentionEstimator::get_gait_phase(double t)
{
    if (!intention_done)
        get_intention();
    intention_done = false;

    float v = std::sqrt(
        std::pow((state_filter.state(s["dly"], 0) + state_filter.state(s["dry"], 0)) / 2, 2) 
        + std::pow((state_filter.state(s["dlz"], 0) + state_filter.state(s["drz"], 0)) / 2, 2)
    );
    float w = state_filter.state(s["do"], 0);
    float left_h = state_filter.state(s["lx"], 0);
    float right_h = state_filter.state(s["rx"], 0);
    float left_v = state_filter.state(s["dlz"], 0);
    float right_v = state_filter.state(s["drz"], 0);

    bool v_move = v > thres_cv;                            // 位置是否明显变化
    bool w_move = std::abs(w) > thres_w;                   // 朝向是否有明显变化
    bool left_move = std::abs(left_v) > thres_lv;          // 左脚是否有明显运动
    bool right_move = std::abs(right_v) > thres_lv;        // 右脚是否有明显运动

    int det = 0;
    // 人体中心没有明显运动#
    if (!v_move && !w_move)
        det = gait_phase_det["stop"];
    // 左右脚没有明显运动
    else if (!left_move && !right_move)
        det = gait_phase_det["support"];
    // 左脚深度变化速度大于右脚
    else if (std::abs(left_v) - std::abs(right_v) > 5)
        det = gait_phase_det["left_swing"];
    // 右脚深度变化速度大于左脚
    else if (std::abs(left_v) - std::abs(right_v) < -5)
        det = gait_phase_det["right_swing"];
    // 左脚高于右脚
    else if (left_h - right_h > 20)
         det = gait_phase_det["left_swing"];
    // 右脚高于左脚
    else if (left_h - right_h < -20)
        det = gait_phase_det["right_swing"];
    else
        det = gait_phase_det["error"];  // 没有产生检测

    if (!det_missing)
    {
        gaitphase_filter.cal(det, det != gait_phase_det["error"]);
        std::cout << intention_filter.intention << std::endl;
        if (gaitphase_filter.gait_cur != gait_phase["stop"])
        {
            std::vector<float> state, orient;
            for (int i = 0; i < state_filter.state.rows(); i++)
            {
                state.push_back(state_filter.state(i, 0));
            }
            for (int i = 0; i < orient_filter.state.rows(); i++)
            {
                orient.push_back(orient_filter.state(i, 0));
            }
            gaitparam_calculator.cal(t, state, orient, gaitphase_filter.gait_cur);
        }
    }

    return gaitphase_filter.gait_cur;
}

int IntentionEstimator::cal(cv::Point3f j0, cv::Point3f j2, cv::Point3f j3, cv::Point3f j5)
{
    int ret = info["good"];
    float temp;
    temp = j0.x * 1000; j0.x = j0.y * 1000; j0.y = temp; j0.z = -j0.z * 1000;
    temp = j2.x * 1000; j2.x = j2.y * 1000; j2.y = temp; j2.z = -j2.z * 1000;
    temp = j3.x * 1000; j3.x = j3.y * 1000; j3.y = temp; j3.z = -j3.z * 1000;
    temp = j5.x * 1000; j5.x = j5.y * 1000; j5.y = temp; j5.z = -j5.z * 1000;
    if (j0.z > 0 && j2.z > 0 && j3.z > 0 && j5.z > 0)
    {
        det_missing = false;
        std::vector<float> loc{(j0.y + j5.y)/2, (j0.z + j5.z)/2};
        std::vector<float> v{0, 0};
        if (loc_pre.size() == 2)
        {
            v[0] = loc[0] - loc_pre[0];
            v[1] = loc[1] - loc_pre[1];
        }

        std::vector<float> v_orient{(float)(std::atan2(v[1], v[0]) + M_PI / 2)};
        int npi = 0;
        if (state_filter.is_init)
        {
            npi = std::round(std::abs(state_filter.state(s["vo"], 0) - v_orient[0]) / M_PI);
            int sign = (state_filter.state(s["vo"], 0) - v_orient[0] > 0) ? 1 :
                ((state_filter.state(s["vo"], 0) - v_orient[0] < 0) ? -1 : 0);
            v_orient[0] += sign * npi * M_PI;
        }
        loc_pre = loc;
        
        std::vector<float> orient{(float)(std::atan2(j2.z - j3.z, j2.y - j3.y) + M_PI / 2)};
        npi = 0;
        if (state_filter.is_init)
        {
            npi = std::round(std::abs(state_filter.state(s["o"], 0) - orient[0]) / M_PI);
            int sign = (state_filter.state(s["o"], 0) - orient[0] > 0) ? 1 :
                ((state_filter.state(s["o"], 0) - orient[0] < 0) ? -1 : 0);
            orient[0] += sign * npi * M_PI;
        }

        std::vector<float> leg;
        if (npi % 2 > 0)
            leg = std::vector<float>{j0.x, j0.y, j0.z, j5.x, j5.y, j5.z};
        else
            leg = std::vector<float>{j5.x, j5.y, j5.z, j0.x, j0.y, j0.z};
        
        std::vector<float> pos;
        pos.insert(pos.end(), leg.begin(), leg.end());
        pos.insert(pos.end(), v_orient.begin(), v_orient.end());
        pos.insert(pos.end(), orient.begin(), orient.end());
        
        float d = state_filter.gating_distance(pos);
        if (d < chi2inv95[8] * thres_valid || ~state_filter.is_init)
        {
            std::cout << "\033[1;32m" << "valid" << "\033[0m" << std::endl;
            ret = info["good"];
            state_filter.cal(pos, true);
            frame_loss = 0;
            if (npi % 2 > 0)
                frame_flip++;
            else
                frame_flip = 0;

            if (frame_flip >= 10)
            {
                std::cout << "\033[1;33m" << "reinitiate" << "\033[0m" << std::endl;
                ret = info["warning"];
                frame_flip = 0;
                state_filter.is_init = false;
            }
        }
        else
        {
            std::cout << "\033[1;31m" << "missing1" << "\033[0m" << std::endl;
            ret = info["error"];
            state_filter.cal(pos, false);
            frame_loss++;
            if (frame_loss >= 20)
            {
                std::cout << "\033[1;33m" << "reinitiate" << "\033[0m" << std::endl;
                ret = info["warning"];
                frame_loss = 0;
                state_filter.is_init = false;
            }
        }
    }
    else
    {
        det_missing = true;
        std::cout << "\033[1;31m" << "missing2" << "\033[0m" << std::endl;
        ret = info["error"];
    }
    return ret;
}