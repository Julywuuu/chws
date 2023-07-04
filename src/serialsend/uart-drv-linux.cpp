#include "uart-drv-linux.h"
#include "uart.h"

UartDrvLinux::UartDrvLinux(std::string uart, int baudrate, E_UART_DATABIT databit, E_UART_STOPBIT stopbit, E_UART_PARITY parity)
{
    openUart(uart, baudrate, databit, stopbit, parity);
}

UartDrvLinux::~UartDrvLinux()
{
    closeUart();
}

bool UartDrvLinux::openUart(std::string uart, int baudrate, E_UART_DATABIT databit, E_UART_STOPBIT sb, E_UART_PARITY parity)
{
    char parityBit;
    int stopBit;

    switch(parity){
        case E_PARITY_EVEN:
            parityBit = EVEN_PARITY;
            break;

        case E_PARITY_ODD:
            parityBit = ODD_PARITY;
            break;

        default:
            parityBit = NONE_PARITY;
            break;
    }

    switch(sb){
        case E_STOPBIT_ONE_STOP:
            stopBit = 1;
            break;

        case E_STOPBIT_TWO_STOP:
            stopBit = 2;
            break;

        default:
            stopBit = 3;
            break;
    }

    fd = ::uart_open((char *)uart.c_str(), baudrate, databit, parityBit, stopBit);
    if(fd < 0){
        std::cout << "uart open error" << std::endl;
        return false;
    }


    return true;
}

void UartDrvLinux::closeUart()
{
    ::close(fd);
}

int UartDrvLinux::readUart(char *buf, int size)
{
    ::uart_read(fd, buf, size);
}

int UartDrvLinux::writeUart(char *buf, int size)
{
    ::uart_write(fd, buf, size);
}

