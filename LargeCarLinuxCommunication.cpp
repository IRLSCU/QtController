//#define CONNECTSELFTEST
#include "LargeCarLinuxCommunication.h"
#include <QDebug>
LargeCarLinuxCommunication::LargeCarLinuxCommunication()
{
    nDeviceType = DEFAULTDEVICETYPE; /* USBCAN-2A或USBCAN-2C或CANalyst-II */
    nDeviceInd = DEFAULTDEVICEIND; /* 第1个设备 */
    nCANInd1=DEFAULTDCANIND1;//第1个通道
    nCANInd2=DEFAULTDCANIND2;//第1个通道
    initConfig();
    initMessageHeader();
}
void LargeCarLinuxCommunication::initConfig(){
    //初始化CAN通道
    vic.AccCode = 0x00600000; //只接收id在0~15的帧
    vic.AccMask = 0x00600000;
    vic.Filter = 1;
    vic.Timing0 = 0x00;//波特率设置
    vic.Timing1 = 0x1C;//波特率设置
    vic.Mode = 0;
}

bool LargeCarLinuxCommunication::connect(){
    DWORD dwRel;
    dwRel = VCI_OpenDevice(nDeviceType,nDeviceInd,0);
    if(dwRel!=1){
        qDebug() << QStringLiteral("打开CAN-USB设备失败，错误代码为") << dwRel << QStringLiteral("(0表示操作失败；-1表示USB-CAN设备不存在或USB掉线)");
        return false;
    }else{
        qDebug() <<QStringLiteral("打开CAN-USB设备成功");
    }

    dwRel = VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd1, &vic);//初始化CAN通道1
    if (dwRel != 1){
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        qDebug() << QStringLiteral("初始化CAN 1 通道错误，错误代码为") << dwRel << QStringLiteral("(0表示操作失败；-1表示USB-CAN设备不存在或USB掉线)");
        return false;
    }else{
        qDebug() <<QStringLiteral("初始化CAN 1 通道成功");
    }
#ifdef CONNECTSELFTEST
    dwRel = VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd2, &vic);//初始化CAN通道2
    if (dwRel != 1){
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        qDebug() <<QStringLiteral("初始化CAN 2 通道错误，错误代码为") << dwRel << QStringLiteral("(0表示操作失败；-1表示USB-CAN设备不存在或USB掉线)");
        return false;
    }else{
        qDebug() <<QStringLiteral("初始化CAN 2 通道成功");
    }
#endif // CONNECTSELFTEST
    /*VCI_BOARD_INFO vbi;
           dwRel = VCI_ReadBoardInfo(nDeviceType, nDeviceInd, &vbi);
           qDebug() << "序列号为:"<<vbi.str_Serial_Num << std::endl;*/
    //清空读写缓冲区
    VCI_ClearBuffer(nDeviceType, nDeviceInd, nCANInd1);
#ifdef CONNECTSELFTEST
    VCI_ClearBuffer(nDeviceType, nDeviceInd, nCANInd2);
#endif // CONNECTSELFTEST
    //启动设备
    if (VCI_StartCAN(nDeviceType, nDeviceInd, nCANInd1) != 1)
    {
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        qDebug() << QStringLiteral("启动设备CAN 1 通道错误，错误代码为") << dwRel << QStringLiteral("(0表示操作失败；-1表示USB-CAN设备不存在或USB掉线)");
        return false;
    }else{
        qDebug() <<QStringLiteral("启动设备CAN 1 通道成功");
    }
#ifdef CONNECTSELFTEST
    if (VCI_StartCAN(nDeviceType, nDeviceInd, nCANInd2) != 1)
    {
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        qDebug() << QStringLiteral("启动设备CAN 2 通道错误，错误代码为" )<< dwRel << QStringLiteral("(0表示操作失败；-1表示USB-CAN设备不存在或USB掉线)");
        return false;
    }else{
        qDebug() <<QStringLiteral("启动设备CAN 2 通道成功");
    }
#endif // CONNECTSELFTEST
    return true;
}
void LargeCarLinuxCommunication::initMessageHeader(){
    vco.ID = 1;//帧ID有可能需要重新设置
    vco.RemoteFlag = 0;
    vco.ExternFlag = 0;
    vco.SendType = 1;//不重发
    vco.DataLen = 8;
}
void LargeCarLinuxCommunication::sendMessage(const unsigned char* message){
    //CAN1发送数据
    QString order;
    for (int j = 0; j < 8; j++) {
        vco.Data[j] =message[j];
        order.append(QString::number(message[j],16));
        order.append(" ");
    }
    //qDebug()<<order;
    DWORD dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd1, &vco, 1);//发送数据
    if(dwRel!=0){
        //qDebug()<<"message send successed";
    }else{
        qDebug()<<"message send failed";
    }
}
void LargeCarLinuxCommunication::receiveMessage(){
#ifdef CONNECTSELFTEST
    //读取CAN2数据
    DWORD dwRel = VCI_Receive(nDeviceType, nDeviceInd, nCANInd2, vcoReadBuffer, 2500, 0);
    //VCI_ClearBuffer(nDeviceType, nDeviceInd, nCANInd2);
#else
    //读取CAN1数据
    DWORD dwRel = VCI_Receive(nDeviceType, nDeviceInd, nCANInd1, vcoReadBuffer, 2500, 0);
    //VCI_ClearBuffer(nDeviceType, nDeviceInd, nCANInd1);
#endif // CONNECTSELFTEST
    if (dwRel > 0) {
        qDebug()<<"has revecived message:"<< dwRel;
        /* 数据处理 */
        for (DWORD i = 0; i < dwRel; i++) {
            if (vcoReadBuffer[i].ID == 2 || vcoReadBuffer[i].ID == 1) {//仅打印有效信息
                qDebug() << "Big car orders' ID:" << vcoReadBuffer[i].ID << "  Data:";
                QString order;
                for (int j = 0; j < vcoReadBuffer[i].DataLen; j++) {
                    order.append(QString::number(vcoReadBuffer[i].Data[j],16));
                    order.append(" ");
                }
                qDebug()<<order;
            }
        }
    }
    else if (dwRel == -1) {
        qDebug() << QStringLiteral("读取数据时USB-CAN设备不存在或USB掉线");
        /* USB-CAN设备不存在或USB掉线，可以调用VCI_CloseDevice并重新VCI_OpenDevice。如此可以达到USB-CAN设备热插拔的效果。 */
    }
}
bool LargeCarLinuxCommunication::close(){
    DWORD  dwRel = VCI_CloseDevice(nDeviceType, nDeviceInd);
    if (dwRel != 1){
        qDebug() << QStringLiteral("关闭设备失败，错误代码为") << dwRel << QStringLiteral("(0表示操作失败；-1表示USB-CAN设备不存在或USB掉线)");
        return false;
    }else{
        qDebug() << "关闭CAN设备成功";
        return true;
    }
}
