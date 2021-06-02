#include "LocationBufferConsumInitRouteThread.h"
#include <QDebug>
LocationBufferConsumInitRouteThread::LocationBufferConsumInitRouteThread(LocationRingBuffer *locationRingBuffer, QObject *parent = 0) : QThread(parent)
{
    this->locationRingBuffer = locationRingBuffer;
    m_startInit = false;
}

LocationBufferConsumInitRouteThread::~LocationBufferConsumInitRouteThread()
{
    qDebug() << "LocationBufferConsumInitRouteThread for initing route has been destoried";
}

void LocationBufferConsumInitRouteThread::stopImmediately()
{
    QMutexLocker locker(&m_lock);
    m_isCanRun = false;
    qDebug() << "stop init";
    locker.unlock();
}
void LocationBufferConsumInitRouteThread::initSignal(bool signal)
{
    QMutexLocker locker(&m_Initlock);
    qDebug() << "initSignal " << (signal ? "start" : "end");
    m_startInit = signal;
    locker.unlock();
}
void LocationBufferConsumInitRouteThread::run()
{
    qDebug() << "GpsBufferReadInitRouteThread for initing route has started";
    m_isCanRun = true;
    //int i=0;
    while (true)
    {
        LocationPosition location;
        if (locationRingBuffer->pop(location))
        { //一直在读取里面的数据，相当于在start线程的时候会把缓冲区清空；当开始初始化后，会将LocationPosition传到初始化的list中
            //qDebug()<<(++i);
            //gpsInfo.printInfo();
            emit sendLocationPointInfo(QPointF(location.x, location.y)); //绘图
            if (m_startInit)
            { //双重判断，由于加锁费时间
                QMutexLocker locker2(&m_Initlock);
                if (m_startInit)
                {   //同时将数据发往初始化列表
                    //                    qDebug()<<"start init route";
                    emit sendInitLocationInfo(location.toString());
                }
                locker2.unlock();
            }
        }
        else
        {
            this->msleep(LOCATIONBUFFERCONSUMINITROUTETHREAD_BOLCKTIME); //阻塞否则cpu占用太高
        }
        QMutexLocker locker(&m_lock);
        if (!m_isCanRun) //在每次循环判断是否可以运行，如果不行就退出循环
        {
            return;
        }
        else
        {
            locker.unlock();
        }
    }
}
