﻿#ifndef GPSBUFFERWRITETHREAD_H
#define GPSBUFFERWRITETHREAD_H
#define GPSBUFFERWIRTETHREAD_BOLCKTIME 50

#include<QThread>
#include<QMutex>
#include "RingBuffer.h"
#include "nmeaPres.h"
#include "ControlMessageStruct.h"
/**
 * @brief The GpsBufferWriteThread class
 * receive the character from gps info ringBuffer
 * and process the char to gps info(nema0183)
 * and store in message ringBuffer
 */
class GpsBufferWriteThread : public QThread{
    Q_OBJECT
private:
    CharRingBuffer* charRingBuffer;
    GpsRingBuffer* gpsRingBuffer;
    QString name;
    NmeaPres* NewParser;
    QMutex m_lock;
    bool m_isCanRun;

    bool isGGA(QString s);//判断是否是GGA
public:
    GpsBufferWriteThread(CharRingBuffer*,GpsRingBuffer*,QObject *parent ,QString name);//传入char循环队列与gps循环队列的指针
    ~GpsBufferWriteThread();
    void stopImmediately();
    void run();

signals:

};
#endif // GpsBufferWriteThread_H
