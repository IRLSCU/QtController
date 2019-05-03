#ifndef GPSBUFFERREADTHREAD_H
#define GPSBUFFERREADTHREAD_H
#define GPSBUFFERREADTHREAD_BOLCKTIME 50
#include "RingBuffer.h"
#include<QThread>
#include<QMutex>
/**
 * @brief The GpsBufferReadThread class
 * 暂未使用
 */
class GpsBufferReadThread : public QThread
{
    Q_OBJECT
private:

    GpsRingBuffer* gpsRingBuffer;
    QMutex m_lock;
    bool m_isCanRun;

public:
    GpsBufferReadThread(GpsRingBuffer*,QObject *parent);
    ~GpsBufferReadThread();
    void stopImmediately();
    void run();

signals:
};

#endif // GPSBUFFERREADTHREAD_H
