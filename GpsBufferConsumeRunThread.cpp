#include "GpsBufferConsumeRunThread.h"
#include <QDebug>
GpsBufferConsumeRunThread::GpsBufferConsumeRunThread(GpsRingBuffer* gpsRingBuffer,QObject *parent=0):QThread(parent){
    this->gpsRingBuffer=gpsRingBuffer;
    m_startInit=false;
}

GpsBufferConsumeRunThread::~GpsBufferConsumeRunThread(){
    qDebug()<<"GpsBufferReadInitRouteThread for initing route has been destoried";
}

void GpsBufferConsumeRunThread::stopImmediately(){
    QMutexLocker locker(&m_lock);
    m_isCanRun = false;
    locker.unlock();
}
void GpsBufferConsumeRunThread::runSignal(bool signal){
    QMutexLocker locker(&m_Initlock);
    m_startInit=signal;
    locker.unlock();
}
void GpsBufferConsumeRunThread::setSendStartGpsInfoSignal(){
    QMutexLocker locker(&m_sendStartGpsInfoLock);
    m_isSendStartGpsInfo=true;
    locker.unlock();
}
void GpsBufferConsumeRunThread::run(){
    m_isCanRun=true;
    m_isSendStartGpsInfo=false;
    int i=0;
    while(true){ 
        GpsInfo gpsInfo;
        if(gpsRingBuffer->pop(gpsInfo)){
            qDebug()<<(++i);
            gpsInfo.printInfo();
            emit sendGpsInfo(QPointF(gpsInfo.longitude,gpsInfo.latitude));//绘图
            if(m_startInit){//双重判断，由于加锁费时间
                QMutexLocker locker2(&m_Initlock);
                if(m_startInit){
                    qDebug()<<"start run and process";
                    emit sendRunGpsInfo(gpsInfo);//同时将数据发往ProcessRunDialog
                }
                locker2.unlock();
            }
            if(m_isSendStartGpsInfo){//发送起始坐标给ProcessRunDialog
                QMutexLocker locker3(&m_sendStartGpsInfoLock);
                if(m_isSendStartGpsInfo){
                    qDebug()<<"send start gps info******";
                    emit sendStartGpsInfo(gpsInfo);
                    m_isSendStartGpsInfo=false;
                }
                locker3.unlock();
            }
        }else{
            this->msleep(GPSBUFFERCONSUMERUNTHREAD_BOLCKTIME);//阻塞否则cpu占用太高
        }
        QMutexLocker locker(&m_lock);
        if(!m_isCanRun)//在每次循环判断是否可以运行，如果不行就退出循环
        {
            return;
        }else{
            locker.unlock();
        }
    }
    qDebug()<<"GpsBufferConsumeRunThread for initing route has started";
}
