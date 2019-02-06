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
    locker.unlock();
}
void GpsBufferReadInitRouteThread::startInit(bool signal){
    QMutexLocker locker(&m_Initlock);
    m_startInit=signal;
    locker.unlock();
}
void GpsBufferReadInitRouteThread::run(){
    m_isCanRun=true;
    int i=0;
    while(true){ 
        GpsInfo gpsInfo;
        if(gpsRingBuffer->pop(gpsInfo)){
            qDebug()<<(++i);
            gpsInfo.printInfo();
            emit sendGpsInfo(QPointF(gpsInfo.longitude,gpsInfo.latitude));
            if(m_startInit){//双重判断，由于加锁费时间
                QMutexLocker locker2(&m_Initlock);
                if(m_startInit){
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
