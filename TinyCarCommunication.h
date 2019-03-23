#ifndef TYNECOMMUNICATION_H
#define TYNECOMMUNICATION_H

#define TINYCARORDERLENGTH 10
#define FRAMEHEADFIRST 0xFF
#define FRAMEHEADSECOND 0xFE
#include <QString>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include "AbstractCommunication.h"
#include "ControlCAN.h"
class TinyCarCommunication:public AbstractCommunication
{
private:
    QSerialPort* serialPort;
    void initConfig();
public:
    TinyCarCommunication(QSerialPort*);//波特率
    QString dataBits;//数据位
    QString parity;//奇偶校验位
    QString ptopBit;//设置停止位
    QString flowBit;//流控);
    QSerialPort* getSerialPort();
    bool connect();
    void sendMessage(const unsigned char*);
    void receiveMessage();
    bool close();
    void sendInfo(char* info,int len);
    void sendInfo(const QString &info);
    char convertCharToHex(char ch);
    void convertStringToHex(const QString &str, QByteArray &byteData);
};

#endif // TYNECOMMUNICATION_H
