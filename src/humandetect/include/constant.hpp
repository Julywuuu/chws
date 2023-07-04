#pragma once
#include <vector>
#include <cstdlib>
#include <map>
#include <string>

#define WIDTH 640 
#define HEIGHT 480 
#define FPS 30

#define CONFIDENCE_THRES 0.2
#define GAIT_PARAMETER_PATH "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/data/gait_parameters.txt"
#define JOINTS_DATA_PATH "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/data/joints.txt"
#define JOINTS_PIXEL_PATH "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/data/jointspixel.txt"
#define LOC_DATA_PATH "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/data/loc.txt"
#define FORWARD_OBS_PATH "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/data/forwardobs.txt"
#define BACK_OBS_PATH "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/data/backobs.txt"
#define OBS_PATH "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/data/obs.txt"
#define DEBUG_INFO_PATH "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/data/debuginfo.txt"

#define MODEL_PATH "/home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/src/humandetect/models/resnet34/"

extern float chi2inv95[9];
extern std::map<std::string, int> info;
extern std::map<std::string, int> intention_mode;
extern std::map<int, std::string> intention_mode_inv;
extern std::map<std::string, int> gait_phase_det;
extern std::map<std::string, int> gait_phase;
extern std::map<int, std::string> gait_phase_inv;
extern std::map<std::string, int> s;