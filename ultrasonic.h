#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#define THRESHOLD 300
#include <QDebug>
#include <QSerialPort>
#include <QObject>
#include <QString>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fileoperation.h"
#include <QLatin1Char>
#include <QLatin1String>
/*
Distance of Ultrasonic
*/

class Ultrasonic : public QObject
{
    Q_OBJECT
public:
    int distance; // 20cm - 720cm
    QString stringInfo;  //RECEIVEd data -> string
    QSerialPort* m_serialPort; //串口类 read distance data
    QString d_byte_str;
    QString portName; // 串口Name
    explicit Ultrasonic(QObject *parent = nullptr);
    ~Ultrasonic();
    void openPort();

    void closePort();
//    int calcDistance();
    int getDistance();
    int hexString2Int(QString s);

    bool checkData(QString s);//check the data jiaoyanhe
public slots:
    QString receiveInfo();
    int calcDistance();

signals:
    QString sendStringInfo(QString);
    void sendDistance(bool,int);
};

#endif // ULTRASONIC_H
