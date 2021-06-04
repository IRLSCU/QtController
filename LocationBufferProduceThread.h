#ifndef LOCATIONBUFFERPRODUCETHREAD_H
#define LOCATIONBUFFERPRODUCETHREAD_H
#define LOCATIONBUFFERPRODUCETHREAD_BOLCKTIME 50

#include<QThread>
#include<QMutex>
#include<QFile>
#include "RingBuffer.h"
#include "nmeaPres.h"
#include "CoTrans.h"
/**
 * @brief The GpsBufferWriteThread class
 * receive the character from gps info ringBuffer
 * and process the char to location info(XYZ)
 * and store in message ringBuffer
 *
 * for test
 */
class LocationBufferProduceThread : public QThread{
    Q_OBJECT
private:
    CharRingBuffer* charRingBuffer;
    LocationRingBuffer* loactionRingBuffer;
    QString name;
    NmeaPres* NewParser;
    QMutex m_lock;
    bool m_isCanRun;

    CCoordinate ordinate;

    bool isGGA(QString s);//判断是否是GGA
public:
    LocationBufferProduceThread(CharRingBuffer*,LocationRingBuffer*,QObject *parent,QString name);//传入char循环队列与gps循环队列的指针
    void initCCoordinate();
    ~LocationBufferProduceThread();
    void stopImmediately();
    void run();

signals:

};
#endif // LOCATIONBUFFERPRODUCETHREAD_H
