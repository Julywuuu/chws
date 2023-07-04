sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyTHS0
sudo chmod 777 /dev/ttyTCU0

sudo rm -rf /home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/depth
mkdir /home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/depth
sudo rm -rf /home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/RGB
mkdir /home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/RGB

sudo rm /home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/data/info.txt
sudo rm /home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/forward.avi
sudo rm /home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/back.avi
sudo rm /home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/map.avi

sudo rm -rf /home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/data
mkdir /home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/data

script -f /home/nvidia/gait-recognition-car-based-on-human-following/obs_detect_module/testfile/data/info.txt
