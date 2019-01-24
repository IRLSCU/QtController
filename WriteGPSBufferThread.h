#ifndef WriteGPSBufferThread_H
#define WriteGPSBufferThread_H

#include<QThread>
#include<QMutex>
#include "RingBuffer.h"
#include "nmeaPres.h"
#include "ControlMessageStruct.h"
/**
 * @brief The WriteGPSBufferThread class
 * receive the character from gps info ringBuffer
 * and process the char to gps info(nema0183)
 * and store in message ringBuffer
 */
class WriteGPSBufferThread : public QThread{
    Q_OBJECT
private:
    RingBuffer<QChar, 20480>* ringBuffer;
    RingBuffer<ControlMessageStruct, 1024>* controlMsgBuffer;
    QString name;
    NmeaPres* NewParser;
    QMutex m_lock;
    bool m_isCanRun;

    bool haveStartAndIsGGA(QString s);//判断是否
public:
    WriteGPSBufferThread(RingBuffer<QChar, 20480>* ringBuffer,QObject *parent ,QString name);
    ~WriteGPSBufferThread();
    void stopImmediately();
    void run();

signals:

};
#endif // WriteGPSBufferThread_H
