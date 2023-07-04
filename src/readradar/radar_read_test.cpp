#include "radar_read.h"



unsigned char running = 1;
static void sig_handle(int signo)
{
    printf("program exit, [%s,%s] Receive SIGNAL %d ====== \r\n", __FILE__, __func__, signo);
    running = 0;
    delay(1000);
    // std::exit(1);  // 有可能析构还没结束 就推出了！不行哦
}

int main(){
    signal(SIGINT, sig_handle);
    uint8_t type = 0x0;  // 0x0,/**< serial type.*/
    int model = 1;       // ORADAR_MS200 = 1

    // radar_reader reader("/dev/ttyACM0", 230400, type, model);
    // std::unique_ptr<radar_reader> reader = std::make_unique<radar_reader>("/dev/ttyACM0", 230400, type, model);
    {
        std::unique_ptr<radar_reader> reader(new radar_reader("/dev/ttyACM0", 230400, type, model));
    while (running)
    {
        if(!reader->radar_read()) {
            std::cout<< "failed read radar data!"<< std::endl;
        }
        full_scan_data_st scan_data = reader->full_scan_data;
        for (int i = 0; i < scan_data.vailtidy_point_num; i++)
        {
            printf("[%d: %f, %f] \n", i, (scan_data.data[i].distance * 0.001), scan_data.data[i].angle);
        }
    }
    reader->save_fulldata();
    reader.reset();
    }
    return 0;
    
}