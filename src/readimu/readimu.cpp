#include "readimu.h"

using namespace imu;

static raw_t raw;//CH040的imu信息

readimu::readimu(char *port_device){
    fd = readimu::open_port(port_device);
	imu_data_decode_init();//HI219
};

readimu::~readimu(){
	printf("imu停止采集\n");
    close(fd);
};


/*
 * @brief Open serial port with the given device name
 *
 * @return The file descriptor on success or -1 on error.
 */
int readimu::open_port(char *port_device){
    struct termios options;

    int fd = open(port_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	tcgetattr(fd, &options);

	if (fd == -1)
    {
        perror("open_port: Unable to open SerialPort");
		return(-1);
    }

    if(fcntl(fd, F_SETFL, 0)<0)
	{
		printf("fcntl failed\n");
	}
    else
	{
		fcntl(fd, F_SETFL, 0);
	}
  
	if(isatty(STDIN_FILENO)==0)
	{
		printf("standard input is not a terminal device\n");
	}
	else 
	{
		printf("isatty success!\n");
	}

	bzero(&options,sizeof(options));

	options.c_cflag = B115200 | CS8 | CLOCAL |CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
	return (fd);	
};


/*
	获取当前航向角---HI219
*/
float readimu::get_yaw(void){
	float Eular[3] = {0};
	uint8_t buf[1024];
	ssize_t n = read(fd, buf, sizeof(buf));
	for(int i=0; i<n; i++){
		Packet_Decode(buf[i]);
	}
	get_eular(Eular);
	// printf("欧拉角°(Pitch Roll Yaw):%0.2f %0.2f %0.2f\n",Eular[0], Eular[1], Eular[2]);
	return Eular[2]/180.0*M_PI;
};


/*
	获取当前航向角---CH040
*/
float readimu::get_yaw_CH040(void){
	uint8_t buf[2048] = "";
	ssize_t n = read(fd, buf, sizeof(buf));
	for(int i=0; i<n; i++){
		ch_serial_input(&raw, buf[i]);
	}
	// printf("欧拉角°(Pitch Roll Yaw):%0.2f %0.2f %0.2f\n",  raw.imu[0].eul[0], raw.imu[0].eul[1], raw.imu[0].eul[2]);
	// printf("%-16s%d\r\n",       "timestamp(ms):", raw->imu[0].timestamp);
	return raw.imu[0].eul[2]/180.0*M_PI;
};