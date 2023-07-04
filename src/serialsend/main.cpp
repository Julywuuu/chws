#include <serialsend.h>



int main(int argc, char const *argv[])
{   
    //const char* arg = argv[1];

    std::string portname_bc = "/dev/ttyTHS0";
    std::string portname_ad = "/dev/ttyTCU0";
    int baudrate = 115200;

    serial::serialsend ser_bc(portname_bc, baudrate);
    serial::serialsend ser_ad(portname_ad, baudrate);

    float speed_front[3] = {0.3,0,0};
    float speed_back[3] = {-0.3,0,0};

    float speed_left[3] = {0,0.3,0};
    float speed_right[3] = {0,-0.3,0};


    float speed_leftFront[3] = {0.15,0.15,0};
    float speed_rightback[3] = {-0.215,-0.215,0};

    float speed_rightFront[3] = {0.215,-0.215,0};
    float speed_leftback[3] = {-0.15,0.15,0};

    float stop[3] = {0,0,0};

    float rotate[3] = {0,0,0.8};

    int time_cur = 1;
    char input;
    while (1)
    {
        std::cout<< "请输入运动方向:"<< std::endl;
        std::cin>>input;
        int flag = input-'0';
        switch(flag){
            case 0: 
                ser_ad.send(rotate);
                ser_bc.send(rotate);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 8: 
                ser_ad.send(speed_front);
                ser_bc.send(speed_front);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 2: 
                ser_ad.send(speed_back);
                ser_bc.send(speed_back);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 4: 
                ser_ad.send(speed_left);
                ser_bc.send(speed_left);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 6: 
                ser_ad.send(speed_right);
                ser_bc.send(speed_right);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 7: 
                ser_ad.send(speed_leftFront);
                ser_bc.send(speed_leftFront);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 3: 
                ser_ad.send(speed_rightback);
                ser_bc.send(speed_rightback);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 9: 
                ser_ad.send(speed_rightFront);
                ser_bc.send(speed_rightFront);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 1: 
                ser_ad.send(speed_leftback);
                ser_bc.send(speed_leftback);
                sleep(time_cur);
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            case 5: 
                ser_ad.send(stop);
                ser_bc.send(stop);
                continue;
            default:
                continue;
        }
        
    }
    
    return 0;
}
