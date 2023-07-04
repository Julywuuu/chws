#include "ord_lidar_driver.h"

class radar_reader
{
public:
    full_scan_data_st full_scan_data;
    one_scan_data_st one_scan_data;
private:
    
    int timeout_ms;                 // 阻塞时间
    ordlidar::OrdlidarDriver device;// 初始化一个驱动设备对象
    bool is_block;                  // 是否阻塞读取
    bool is_full;                   // 是否读取数据一圈数据还是一包
    std::string port_name;          // 设备名字  /dev/ttyACM0
    uint32_t serialBaudrate;        // 波特率

public:
    // 0x0,  表示串口通信模式
    // model =1 表示 ORADAR_MS200 = 1 使用MS200
    radar_reader(std::string port_name, uint32_t serialBaudrate, uint8_t type=0x0, int model=1, bool is_block=true, bool is_full=true);    // 构造函数  
    ~radar_reader();                     // 析构函数

    void radar_init();
    void radar_destroy();

    bool radar_read();
    bool save_fulldata();

};


