#ifndef READIMU_H_
#define READIMU_H_

#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>
#include <math.h>

extern "C"{
	#include "packet.h"//HI219
	#include "imu_data_decode.h"//HI219
    #include "ch_serial.h"//CH040
}



namespace imu{

    class readimu
    {
        private:
            int fd;

        public:
            readimu(char *port_device);
            ~readimu();
            
			int open_port(char *port_device);

			float get_yaw(void);

            float get_yaw_CH040(void);

            
    };
    

}


#endif
