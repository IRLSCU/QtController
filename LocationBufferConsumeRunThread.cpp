#include "LocationBufferConsumeRunThread.h"
#include <QDebug>
LocationBufferConsumeRunThread::LocationBufferConsumeRunThread(LocationRingBuffer* locationRingBuffer,QObject *parent=0):QThread(parent){
    this->locationRingBuffer=locationRingBuffer;
    m_startRun=false;
}

LocationBufferConsumeRunThread::~LocationBufferConsumeRunThread(){
    qDebug()<<"LocationBufferConsumeRunThread for initing route has been destoried";
}

void LocationBufferConsumeRunThread::stopImmediately(){
    QMutexLocker locker(&m_lock);
    m_isCanRun = false;
    locker.unlock();
}
void LocationBufferConsumeRunThread::runSignal(bool signal){
    QMutexLocker locker(&m_Initlock);
    m_startRun=signal;
    locker.unlock();
}
void LocationBufferConsumeRunThread::setSendStartLocationSignal(){
    QMutexLocker locker(&m_sendStartLocationLock);
    m_isSendStartLocation=true;
    locker.unlock();
}
void LocationBufferConsumeRunThread::run(){
    m_isCanRun=true;
    m_isSendStartLocation=false;

    //int i=0;
    while(true){ 
        LocationPosition locationPosition;
        if(locationRingBuffer->pop(locationPosition)){
            //qDebug()<<(++i);
            //gpsInfo.printInfo();
            emit sendLocation(QPointF(locationPosition.x,locationPosition.y));//绘图
            if(m_startRun){//双重判断，由于加锁费时间
                QMutexLocker locker2(&m_Initlock);
                if(m_startRun){
                    emit sendRunLocation(locationPosition);//同时将数据发往ProcessRunDialog
                }
                locker2.unlock();
            }
            if(m_isSendStartLocation){//发送起始坐标给ProcessRunDialog
                QMutexLocker locker3(&m_sendStartLocationLock);
                if(m_isSendStartLocation){
                    emit sendStartLocation(locationPosition);
                    m_isSendStartLocation=false;
                }
                locker3.unlock();
            }
        }else{
            this->msleep(LOCATIONBUFFERCONSUMERUNTHREAD_BOLCKTIME);//阻塞否则cpu占用太高
        }
        QMutexLocker locker(&m_lock);
        if(!m_isCanRun)//在每次循环判断是否可以运行，如果不行就退出循环
        {
            return;
        }else{
            locker.unlock();
        }
    }
    qDebug()<<"LocationBufferConsumeRunThread for initing route has started";
}
