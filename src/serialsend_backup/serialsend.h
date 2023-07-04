#ifndef SERIALSEND_H_
#define SERIALSEND_H_

#include <math.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include "stdio.h"
#include "uart-drv-linux.h"




namespace serial{

    // class car_config
    // {
    //     public:
    //         //申明小车最大可达速度范围（单位m/s）
    //         float Vx_MIN = -1.0675;
    //         float Vx_MAX = 1.0675;
    //         //map_x函数和unpack_reply函数所用参数,代表实际速度和输入值之间的转换系数,由实验测得
    //         //使用该系数使输入用实际速度表示，更直观
    //         //后续可使用四轮全向小车运动学模型,将x,y,z轴的速度转换为四轮的速度,发给下位机
    //         //让嵌入式工程师配合实现更精确的运动(PID控制电机转速...)
    //         float Map_x_k = 0.0327;//0.035918208221570;

    //         float Vy_MIN = -0.61;//
    //         float Vy_MAX = 0.61;//
    //         float Map_y_k = 0.019568012322921;//0.020512820512821;

    //         //自转速度（单位rad/s）
    //         float Vz_MIN = -M_PI/3;
    //         float Vz_MAX = M_PI/3;
    //         float Map_z_k = 0.035103221426921;
    // };

        class car_config
    {
        public:
            //申明小车最大可达速度范围（单位m/s）
            float Vx_MIN = -0.53;
            float Vx_MAX = 0.53;
            //map_x函数和unpack_reply函数所用参数,代表实际速度和输入值之间的转换系数,由实验测得
            //使用该系数使输入用实际速度表示，更直观
            //后续可使用四轮全向小车运动学模型,将x,y,z轴的速度转换为四轮的速度,发给下位机
            //让嵌入式工程师配合实现更精确的运动(PID控制电机转速...)
            float Map_x_k = 0.0327;//0.035918208221570;

            float Vy_MIN = -0.53;//
            float Vy_MAX = 0.53;//
            float Map_y_k = 0.019568012322921;//0.020512820512821;

            //自转速度（单位rad/s）
            float Vz_MIN = -M_PI/3;
            float Vz_MAX = M_PI/3;
            float Map_z_k = 0.035103221426921;
    };

    class serialsend
    {
        private:
            car_config _car;
            UartDrvLinux * uart;
        public:
            serialsend(std::string portx,int baudrate);
            ~serialsend();
            
            unsigned char getBcc(unsigned char *data, int length);
            char * setXyz(unsigned short x, unsigned short y, unsigned short z);
            //发包时所有的数都要经以下函数转化成整型数之后再发给电机。
            int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
            //收包时所有的数都要经以下函数转化成浮点型。
            float uint_to_float(int x_int, float x_min, float x_max, int bits);
            
            void map_xyz(float * truespeedx, float * truespeedy, float * truespeedz);

            char * DataTrans(float * data);

            //接收例程代码（将收到的16进制数据转化为单位为m/s的各轴速度值）
            void unpack_reply(char * msg);

            void send(float * speed);





    };
    

}


#endif