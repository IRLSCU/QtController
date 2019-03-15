#ifndef GPSBUFFERCONSUMERUNTHREAD_H
#define GPSBUFFERCONSUMERUNTHREAD_H
#define GPSBUFFERCONSUMERUNTHREAD_BOLCKTIME 50
#include "RingBuffer.h"
#include "GPSProcesser.h"
#include "PID.h"
#include<QThread>
#include<QMutex>
#include<QList>
/**
 * @brief The GpsBufferConsumeRunThread class
 * consume gpsBuffer to init route thread
 * 消费者线程、开始消费GpsInfo信息
 */
class GpsBufferConsumeRunThread : public QThread
{
    Q_OBJECT
private:
    GpsRingBuffer* gpsRingBuffer;
    QMutex m_lock;
    QMutex m_Initlock;
    QMutex m_sendStartGpsInfoLock;
    bool m_isCanRun;//控制线程开闭
    bool m_startInit;//是否开始RUN
    bool m_isSendStartGpsInfo;

public:
    GpsBufferConsumeRunThread(GpsRingBuffer*,QObject *parent);
    ~GpsBufferConsumeRunThread();
    void stopImmediately();
    void runSignal(bool signal);
    void setSendStartGpsInfoSignal();
    void run();

signals:
    void sendGpsInfo(QPointF point);//每接收一个点就发送，发送给paintWeidge绘制
    void sendRunGpsInfo(GpsInfo point);//当开始初始化路径时(m_startInit=true)，一直发送gps信息
    void sendStartGpsInfo(GpsInfo point);//m_isSendStartGpsInfo=true,每次只发送一次后，m_isSendStartGpsInfo=false
};

#endif // GPSBUFFERCONSUMERUNTHREAD_H
