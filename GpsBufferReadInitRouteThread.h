#ifndef GPSBUFFERREADINITROUTETHREAD_H
#define GPSBUFFERREADINITROUTETHREAD_H
#define GPSBUFFERREADINITROUTETHREAD_BOLCKTIME 50
#include "RingBuffer.h"
#include<QThread>
#include<QMutex>
#include<QList>
/**
 * @brief The GpsBufferReadInitRouteThread class
 * consume gpsBuffer to init route thread
 * 消费者线程去初始化路径
 */
class GpsBufferReadInitRouteThread : public QThread
{
    Q_OBJECT
private:
    GpsRingBuffer* gpsRingBuffer;
    QMutex m_lock;
    QMutex m_Initlock;
    bool m_isCanRun;
    bool m_startInit;

public:
    GpsBufferReadInitRouteThread(GpsRingBuffer*,QObject *parent);
    ~GpsBufferReadInitRouteThread();
    void stopImmediately();
    void startInit(bool signal);
    void run();

signals:
    void sendGpsInfo(QPointF point);//发送给paintWeidge绘制
    void sendInitGpsInfo(GpsInfo point);//开始初始化路径时，发送gps信息
};

#endif // GPSBUFFERREADINITROUTETHREAD_H
