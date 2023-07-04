#include "constant.hpp"

float chi2inv95[9] = {
    3.8415,
    5.9915,
    7.8147,
    9.4877,
    11.070,
    12.592,
    14.067,
    15.507,
    16.919
};

std::map<std::string, int> info = {
    {"good", 0},
    {"warning", 1},
    {"error", 2}
};

std::map<std::string, int> intention_mode = {
    {"stop", 0},
    {"forward", 1},
    {"lateral", 2},
    {"turn", 3}
};

std::map<int, std::string> intention_mode_inv = {
    {0, "stop"},
    {1, "forward"},
    {2, "lateral"},
    {3, "turn"}
};

std::map<std::string, int> gait_phase_det = {
    {"stop", 0},
    {"left_swing", 1},
    {"right_swing", 2},
    {"support", 3},
    {"error", 4}
};

std::map<std::string, int> gait_phase = {
    {"stop", 0},
    {"left_swing", 1},
    {"left_support", 2},
    {"right_swing", 3},
    {"right_support", 4}
};

std::map<int, std::string> gait_phase_inv = {
    {0, "stop"},
    {1, "left_swing"},
    {2, "left_support"},
    {3, "right_swing"},
    {4, "right_support"}
};

std::map<std::string, int> s = {
    {"lx", 0},
    {"ly", 1},
    {"lz", 2},
    {"rx", 3},
    {"ry", 4},
    {"rz", 5},
    {"vo", 6},
    {"o", 7},
    {"dlx", 8},
    {"dly", 9},
    {"dlz", 10},
    {"drx", 11},
    {"dry", 12},
    {"drz", 13},
    {"dvo", 14},
    {"do", 15}
};