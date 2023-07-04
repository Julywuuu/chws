#ifndef _UART_DRV_LINUX_H_
#define _UART_DRV_LINUX_H_
#include <iostream>

extern "C"{
    #include "uart.h"
}

typedef enum {
    E_UART_OK = 0,
    E_UART_ERR_NO_SUCH_UART,
    E_UART_ERR_INVALID_BITRATE,
    E_UART_ERR_DRV_NOT_INSTALL,
    E_UART_ERR_START_FAILED,
    E_UART_ERR_UNKNOW,
}E_UART_ERR_CODE;

typedef enum{
    E_DATABIT_5 = 5,
    E_DATABIT_6,
    E_DATABIT_7,
    E_DATABIT_8,
}E_UART_DATABIT;

typedef enum{
    E_STOPBIT_ONE_STOP,
    E_STOPBIT_ONE_AND_A_HALF_STOP,
    E_STOPBIT_TWO_STOP,
}E_UART_STOPBIT;

typedef enum{
    E_PARITY_ODD = 0,
    E_PARITY_EVEN,
    E_PARITY_NONE,
}E_UART_PARITY;

class UartDrvLinux{

public:
    UartDrvLinux(std::string uart, int baudrate, E_UART_DATABIT databit, E_UART_STOPBIT stopbit, E_UART_PARITY parity);
    virtual ~UartDrvLinux();
    bool openUart(std::string uart, int baudrate, E_UART_DATABIT databit, E_UART_STOPBIT stopbit, E_UART_PARITY parity);
    void closeUart();
    int readUart(char *buf, int size);
    int writeUart(char *buf, int size);

private:
    int fd;



};


#endif
