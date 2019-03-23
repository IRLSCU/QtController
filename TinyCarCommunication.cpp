#define CONNECTSELFTEST

#include "TinyCarCommunication.h"

#include <QDebug>
TinyCarCommunication::TinyCarCommunication(QSerialPort* serialPort){
    this->serialPort=serialPort;
}
void TinyCarCommunication::initConfig(){

}

bool TinyCarCommunication::connect(){
    if(serialPort->open(QSerialPort::OpenModeFlag::ReadWrite)){
        return true;
    }else{
        qDebug()<<"TinyCarCommunication opened failed";
        return false;
    }

}

void TinyCarCommunication::sendMessage(const unsigned char* message){
    sendInfo(message,TINYCARORDERLENGTH);
}
void TinyCarCommunication::receiveMessage(){
    QByteArray readComData = serialPort->readAll();
    qDebug()<<readComData;
}
bool TinyCarCommunication::close(){
    if(serialPort->isOpen()){
        serialPort->clear();
        serialPort->close();
    }
    qDebug()<<serialPort->portName()<<"has been closed";
}
void TinyCarCommunication::convertStringToHex(const QString &str, QByteArray &byteData)
{
    int hexdata,lowhexdata;
    int hexdatalen = 0;
    int len = str.length();
    byteData.resize(len/2);
    char lstr,hstr;
    for(int i=0; i<len; )
    {
        //char lstr,
        hstr=str[i].toLatin1();
        if(hstr == ' ')
        {
            i++;
            continue;
        }
        i++;
        if(i >= len)
            break;
        lstr = str[i].toLatin1();
        hexdata = convertCharToHex(hstr);
        lowhexdata = convertCharToHex(lstr);
        if((hexdata == 16) || (lowhexdata == 16))
            break;
        else
            hexdata = hexdata*16+lowhexdata;
        i++;
        byteData[hexdatalen] = (char)hexdata;
        hexdatalen++;
    }
    byteData.resize(hexdatalen);
}

//另一个 函数 char 转为 16进制
char TinyCarCommunication::TinyCarCommunication::convertCharToHex(char ch)
{
    /*
        0x30等于十进制的48，48也是0的ASCII值，，
        1-9的ASCII值是49-57，，所以某一个值－0x30，，
        就是将字符0-9转换为0-9

        */
    if((ch >= '0') && (ch <= '9'))
        return ch-0x30;
    else if((ch >= 'A') && (ch <= 'F'))
        return ch-'A'+10;
    else if((ch >= 'a') && (ch <= 'f'))
        return ch-'a'+10;
    else return (-1);
}
void TinyCarCommunication::TinyCarCommunication::sendInfo(char* info,int len){

    for(int i=0; i<len; ++i)
    {
        printf("0x%x\n", info[i]);
    }
    serialPort->write(info,len);//这句是真正的给单片机发数据 用到的是QIODevice::write 具体可以看文档
}

void TinyCarCommunication::TinyCarCommunication::sendInfo(const QString &info){

    QByteArray sendBuf;
    if (info.contains(" "))
    {
        info.replace(QString(" "),QString(""));//我这里是把空格去掉，根据你们定的协议来
    }
    qDebug()<<"Write to serial: "<<info;
    convertStringToHex(info, sendBuf); //把QString 转换 为 hex
    serialPort->write(sendBuf);////这句是真正的给单片机发数据 用到的是QIODevice::write 具体可以看文档
}

