#include "readimu.h"

int main(int argc, char *argv[])
{
    imu::readimu imudata("/dev/ttyUSB1");//初始化imu对象
    int clcye_times = 0;
    while(1){
        clcye_times++;

        // float yaw = imudata.get_yaw();
        // printf("航向角rad = %0.4f\n",yaw);

        float yaw = imudata.get_yaw_CH040();
        printf("航向角rad = %0.4f\n",yaw);

        usleep(20*1000);
        if(clcye_times == 1000){
            break;
        }
    }

    return 0;
}
