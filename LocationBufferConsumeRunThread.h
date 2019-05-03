#ifndef LOCATIONBUFFERCONSUMERUNTHREAD_H
#define LOCATIONBUFFERCONSUMERUNTHREAD_H
#define LOCATIONBUFFERCONSUMERUNTHREAD_BOLCKTIME 50
#include "RingBuffer.h"
#include "GPSProcesser.h"
#include "PID.h"
#include<QThread>
#include<QMutex>
#include<QList>
/**
 * @brief
 * 消费者线程、开始消费location信息
 */
class LocationBufferConsumeRunThread : public QThread
{
    Q_OBJECT
private:
    LocationRingBuffer* locationRingBuffer;
    QMutex m_lock;
    QMutex m_Initlock;
    QMutex m_sendStartLocationLock;
    bool m_isCanRun;//控制线程开闭
    bool m_startRun;//是否开始初始化路径
    bool m_isSendStartLocation;

public:
    LocationBufferConsumeRunThread(LocationRingBuffer*,QObject *parent);
    ~LocationBufferConsumeRunThread();
    void stopImmediately();
    void runSignal(bool signal);
    void setSendStartLocationSignal();
    void run();

signals:
    void sendLocation(QPointF point);//每接收一个点就发送，发送给paintWeidge绘制
    void sendRunLocation(LocationPosition point);//当开始初始化路径时(m_startInit=true)，一直发送gps信息
    void sendStartLocation(LocationPosition location);//m_isSendStartLocation=true,m_isSendStartLocation=false
};

#endif // LOCATIONBUFFERCONSUMERUNTHREAD_H
