#include "serialsend.h"

using namespace std;

using namespace serial;


serialsend::serialsend(string portx,int baudrate=115200){
    uart = new UartDrvLinux(portx, baudrate, E_DATABIT_8, E_STOPBIT_ONE_STOP, E_PARITY_NONE);
};

serialsend::~serialsend(){
    float stop[3] = {0,0,0}; 
    this->send(stop);
};


unsigned char serialsend::getBcc(unsigned char *data, int length){
    unsigned char i;
    unsigned char bcc = 0;        // Initial value
#if 1
    while(length--)
    {
        bcc ^= *data++;
    }
#else
    for ( i = 0; i < length; i++ )
    {
        bcc ^= data[i];        // crc ^= *data;
    }
#endif
    return bcc;
};


char * serialsend::setXyz(unsigned short x, unsigned short y, unsigned short z){
    //if(x < 0){x = x + (1 << 16); }
    static char cmd[] = {0x0b, 0x00, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7B, 0x0d};
    static unsigned char xh,xl,yh,yl,zh,zl;

    xh = (x & 0xff00) >> 8;
    xl = (x & 0x00ff) >> 0;
    yh = (y & 0xff00) >> 8;
    yl = (y & 0x00ff) >> 0;
    zh = (z & 0xff00) >> 8;
    zl = (z & 0x00ff) >> 0;

    cmd[3] = xh;
    cmd[4] = xl;
    cmd[5] = yh;
    cmd[6] = yl;
    cmd[7] = zh;
    cmd[8] = zl;

    cmd[9] = getBcc((unsigned char*)cmd, 9);

    return cmd; 
};


//发包时所有的数都要经以下函数转化成整型数之后再发给电机。
int serialsend::float_to_uint(float x, float x_min, float x_max, unsigned int bits){
    /// Converts a float to an unsigned int, given range and number of bits /// 
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    int re = (int)( x * ((float)(((1 << bits) - 1) / span)));
    return re;
};


//收包时所有的数都要经以下函数转化成浮点型。
float serialsend::uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits /// 
    float span = x_max - x_min;
    float offset = 0;
    if(x_int >= (1 << bits)*0.5)
    {
        x_int = x_int - ((1 << bits) - 1);
    }
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
};


void serialsend::map_xyz(float * truespeedx, float * truespeedy, float * truespeedz){
    *truespeedx = this->_car.Map_x_k*(*truespeedx);
    *truespeedy = this->_car.Map_y_k*(*truespeedy);
    *truespeedz = this->_car.Map_z_k*(*truespeedz);
};


char * serialsend::DataTrans(float * data){
    //这些设定的电机参数，根据他们生成待发送数据
	int vx_int,vy_int,vz_int;
	float vx_des,vy_des,vz_des;
	///（1） limit data to be within bounds ///
    vx_des = fminf(fmaxf(this->_car.Vx_MIN, data[0]), this->_car.Vx_MAX);
    vy_des = fminf(fmaxf(this->_car.Vy_MIN, data[1]), this->_car.Vy_MAX);
    vz_des = fminf(fmaxf(this->_car.Vz_MIN, data[2]), this->_car.Vz_MAX);
    
    //速度受限提醒
    if (vx_des != data[0]){cout << "Warning!---Vx速度已达极值"<< endl;}
    if (vy_des != data[1]){cout << "Warning!---Vy速度已达极值"<< endl;}
    if (vz_des != data[2]){cout << "Warning!---Vz速度已达极值"<< endl;}

    //根据实际测量得到的映射函数(后续若有实际对应关系也可直接用)
    map_xyz(&vx_des,&vy_des,&vz_des);

    /// （2）convert floats to unsigned ints /// 
    vx_int = float_to_uint(vx_des, this->_car.Vx_MIN, this->_car.Vx_MAX, 16);
    vy_int = float_to_uint(vy_des, this->_car.Vy_MIN, this->_car.Vy_MAX, 16);
    vz_int = float_to_uint(vz_des, this->_car.Vz_MIN, this->_car.Vz_MAX, 16);
    
    char * transed = setXyz(vx_int,vy_int,vz_int);
    //printf("allocated address: %p\n", transed);////
    return transed;
};


//接收例程代码（将收到的16进制数据转化为单位为m/s的各轴速度值）
void serialsend::unpack_reply(char * msg){
    /// unpack ints from can buffer /// 
	float vx,vy,vz;
	int vx_int,vy_int,vz_int;
    vx_int = (msg[3] << 8) | msg[4]; 			
    vy_int = (msg[5] << 8) | msg[6]; 		 
    vz_int = (msg[7] << 8) | msg[8];		
    /// convert ints to floats /// 
    vx = uint_to_float(vx_int, this->_car.Vx_MIN, this->_car.Vx_MAX, 16)/this->_car.Map_x_k;
    vy = uint_to_float(vy_int, this->_car.Vy_MIN, this->_car.Vy_MAX, 16)/this->_car.Map_y_k;
    vz = uint_to_float(vz_int, this->_car.Vz_MIN, this->_car.Vz_MAX, 16)/this->_car.Map_z_k;
    cout << "send:" << std::endl;
    cout << "Vx:"<< vx << std::endl;
    cout << "Vy:"<< vy << std::endl;
    cout << "Vz:"<< vz << std::endl;
};


//发送函数
void serialsend::send(float * speed){
    char *ptr = DataTrans(speed);
    // //验证发送数据
    // for(int i=0; i<11; i++){    
    //     cout << hex << int(ptr[i])<<"  "; 
    //     if(i==10) cout<<" "<<endl;   
    // }
    uart->writeUart(ptr, 11);
    //验证接收函数
    //unpack_reply(ptr);
};


// int main(int argc, const char **argv)
// {
    
//     // string portx = "/dev/ttyTHS1";
//     // int baudrate = 115200;
//     serialsend ser("/dev/ttyTHS1",115200);
//     float data[3] = {0.1, 0.0, 0.25*M_PI};
//     ser.send(data);

//     return 0;s
// }
























