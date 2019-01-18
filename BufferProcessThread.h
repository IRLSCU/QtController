#ifndef BUFFERPROCESSTHREAD_H
#define BUFFERPROCESSTHREAD_H

#include<QThread>
#include<QMutex>
#include "RingBuffer.h"
#include "nmeaPres.h"
class BufferProcessThread : public QThread{
private:
    RingBuffer<QChar, 20480>* ringBuffer;
    QString name;
    NmeaPres* NewParser;
    QMutex m_lock;
    bool m_isCanRun;

    bool haveStartAndIsGGA(QString s);//判断是否
public:
    BufferProcessThread(RingBuffer<QChar, 20480>* ringBuffer,QObject *parent ,QString name);
    ~BufferProcessThread();
    void stopImmediately();
    void run();

};
#endif // BUFFERPROCESSTHREAD_H
