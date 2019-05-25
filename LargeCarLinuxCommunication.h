#ifndef LARGECARLINUXCOMMUNICATION_H
#define LARGECARLINUXCOMMUNICATION_H

#define DEFAULTDEVICETYPE 4;
#define DEFAULTDEVICEIND 0;
#define DEFAULTDCANIND1 0;
#define DEFAULTDCANIND2 1;

#include "AbstractCommunication.h"
#include "ControlCAN.h"

class LargeCarLinuxCommunication:public AbstractCommunication
{
    private:
        int nDeviceType; /* USBCAN-2A或USBCAN-2C或CANalyst-II */
        int nDeviceInd; /* 第几个设备 */
        int nCANInd1;//第1个通道
        int nCANInd2;//第1个通道
        VCI_CAN_OBJ vcoReadBuffer[2500];
        VCI_INIT_CONFIG vic;//通道信息
        VCI_CAN_OBJ vco;//message信息
        VCI_BOARD_INFO vbi;
        void initConfig();
        void initMessageHeader();
    public:
        LargeCarLinuxCommunication();
        bool connect();
        void sendMessage(const unsigned char*);
        void receiveMessage();
        bool close();
};

#endif // LARGECARLINUXCOMMUNICATION_H
