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
    //for test
//    0 0
//    -0.926152 0.52687
//    -1.89345 0.983842
//    -2.89505 1.51831
//    -3.89219 1.8344
//    -4.88134 1.99365
//    -5.8807 2.16939
//    -6.95504 2.43004
//    -7.99796 2.81899
//    -9.05284 3.2754
//    -10.0857 3.77131
//    -11.1463 4.26907
//    -12.2387 4.78593
//    -13.1904 5.27146
//    -14.1344 5.66299
//    -15.0527 6.08826
//    -16.083 6.59901
//    -16.958 7.10011
//    -17.8929 7.66497
//    -18.8343 8.24857
    locationRingBuffer->push(LocationPosition());
    locationRingBuffer->push(LocationPosition(-1,-0.5));
    locationRingBuffer->push(LocationPosition(-3,1.6));
    locationRingBuffer->push(LocationPosition(-10,3.7));

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
