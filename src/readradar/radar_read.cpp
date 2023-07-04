#include "radar_read.h"

radar_reader::radar_reader(std::string port_name, uint32_t serialBaudrate, uint8_t type, int model, bool is_block, bool is_full):
    port_name(port_name), 
    serialBaudrate(serialBaudrate), 
    timeout_ms(1000), 
    device(type, model),
    is_block(is_block),
    is_full(is_full)
    {
    radar_init();
}

radar_reader::~radar_reader(){
    radar_destroy();
}

void radar_reader::radar_init(){
    // 设置串口属性
    device.SetSerialPort(port_name, serialBaudrate);
    if(!device.Connect()) {         // // 检查激光雷达串口并打开，创建激光雷达串口读写线程
        printf("lidar device connect %s fail..\n", port_name.c_str());
    }
    printf("scan_frame_data lidar device connect succuss..\n");
}

/*
    读取数据
*/
bool radar_reader::radar_read(){
    bool ret = false;
    if(is_block) {
        if(is_full) {
            if(device.GrabFullScanBlocking(full_scan_data, this->timeout_ms)) {
                ret = true;
            }
        }
        else{
            if(device.GrabOneScanBlocking(one_scan_data, this->timeout_ms)){
                ret = true;
            }
        }
    }
    else {
        if(is_full) {
            if(device.GrabFullScan(full_scan_data)){
                ret = true;
            }
        }
        else{
            if(device.GrabOneScan(one_scan_data)) {
                ret = true;
            }
        }
    }
    return ret;
}

void radar_reader::radar_destroy(){
    device.Disconnect();
}

// 保存到很多0 是因为full_scan_data.data定义时 point_data_t data[POINT_CIRCLE_MAX_SIZE + 1]; 后面的参数是4097 ，每次都会写4097个点，要是后面全是0，0，0
bool radar_reader::save_fulldata(){
    std::vector<point_data_t> points_cloud;

    // 没有进行有效点的判断。有效点仅有vailtidy_point_num个
    // for(auto point: this->full_scan_data.data) {
    //     points_cloud.push_back(point);
    // }
    for(int i=0; i<full_scan_data.vailtidy_point_num; i++) {
        points_cloud.push_back(full_scan_data.data[i]);
    }

    int fd = open("point_cloud.txt", O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        perror("open");
        return -1;
    }
    int count = 0;
    for(const auto &pt : points_cloud) {
        count++;
        char buf[256];
        memset(buf, 0, sizeof(buf));
        snprintf(buf, sizeof(buf), "%hu %hu %.2f\n", pt.distance, pt.intensity, pt.angle);
        write(fd, buf, strlen(buf));
    }
    cout<< count <<" points cloud write done!"<<endl;
    close(fd);
    return 0;
}