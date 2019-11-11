#ifndef LOCATIONBUFFERCONSUMINITROUTETHREAD_H
#define LOCATIONBUFFERCONSUMINITROUTETHREAD_H
#define LOCATIONBUFFERCONSUMINITROUTETHREAD_BOLCKTIME 50
#include "RingBuffer.h"
#include<QThread>
#include<QMutex>
#include<QList>
/**
 * @brief The GpsBufferReadInitRouteThread class
 * consume gpsBuffer to init route thread
 * 消费者线程去初始化路径
 */
class LocationBufferConsumInitRouteThread : public QThread
{
    Q_OBJECT
private:
    LocationRingBuffer * locationRingBuffer;
    QMutex m_lock;
    QMutex m_Initlock;
    bool m_isCanRun;
    bool m_startInit;

public:
    LocationBufferConsumInitRouteThread(LocationRingBuffer*,QObject *parent);
    ~LocationBufferConsumInitRouteThread();
    void stopImmediately();
    void initSignal(bool signal);
    void run();

signals:
    void sendLocationPointInfo(QPointF point);//发送给paintWeidge绘制
    void sendInitLocationInfo(QString point);//开始初始化路径时，发送gps信息
};

#endif // LOCATIONBUFFERCONSUMINITROUTETHREAD_H
