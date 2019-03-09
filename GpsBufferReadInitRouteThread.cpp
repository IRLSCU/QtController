#include "GpsBufferReadInitRouteThread.h"
#include <QDebug>
GpsBufferReadInitRouteThread::GpsBufferReadInitRouteThread(GpsRingBuffer* gpsRingBuffer,QObject *parent=0):QThread(parent){
    this->gpsRingBuffer=gpsRingBuffer;
    m_startInit=false;
}

GpsBufferReadInitRouteThread::~GpsBufferReadInitRouteThread(){
    qDebug()<<"GpsBufferReadInitRouteThread for initing route has been destoried";
}

void GpsBufferReadInitRouteThread::stopImmediately(){
    QMutexLocker locker(&m_lock);
    m_isCanRun = false;
    qDebug()<<"stop init";
    locker.unlock();
}
void GpsBufferReadInitRouteThread::initSignal(bool signal){
    QMutexLocker locker(&m_Initlock);
    qDebug()<<"initSignal"<<(signal?"start":"end");
    m_startInit=signal;
    locker.unlock();
}
void GpsBufferReadInitRouteThread::run(){
    m_isCanRun=true;
    int i=0;
    while(true){ 
        GpsInfo gpsInfo;
        if(gpsRingBuffer->pop(gpsInfo)){//一直在读取里面的数据，相当于在start线程的时候会把缓冲区清空；当开始初始化后，会将gpsInfo传到初始化的list中
            qDebug()<<(++i);
            gpsInfo.printInfo();
            emit sendGpsInfo(QPointF(gpsInfo.longitude,gpsInfo.latitude));//绘图
            if(m_startInit){//双重判断，由于加锁费时间
                QMutexLocker locker2(&m_Initlock);
                if(m_startInit){//同时将数据发往初始化列表
                    //qDebug()<<"start init";
                    //emit sendInitGpsInfo(gpsInfo);
                    emit sendInitGpsInfo(gpsInfo);
                }
                locker2.unlock();
            }
        }else{
            this->msleep(GPSBUFFERREADINITROUTETHREAD_BOLCKTIME);//阻塞否则cpu占用太高
        }
        QMutexLocker locker(&m_lock);
        if(!m_isCanRun)//在每次循环判断是否可以运行，如果不行就退出循环
        {
            return;
        }else{
            locker.unlock();
        }
    }
    qDebug()<<"GpsBufferReadInitRouteThread for initing route has started";
}
