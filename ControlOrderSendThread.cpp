#include "ControlOrderSendThread.h"
#include <QDebug>
#include <QDateTime>
ControlOrderSendThread::ControlOrderSendThread(QObject *parent=0):QThread(parent){
    m_enable=false;
    communication=CommunicationFactory::createCommunication(LargeCarWindows);
    //todo 打开接口
}

ControlOrderSendThread::~ControlOrderSendThread(){
    //todo 关闭接口
    delete communication;
    qDebug()<<"ControlOrderSendThread for sending car's control orders has been destoried";
}

void ControlOrderSendThread::stopImmediately(){

    QMutexLocker locker(&m_threadRunLock);
    m_isCanRun = false;
    locker.unlock();
}
void ControlOrderSendThread::enableSignal(bool signal){
    QMutexLocker locker(&m_enableLock);
    m_enable=signal;
    locker.unlock();
}

void ControlOrderSendThread::run(){
    qDebug()<<"ControlOrderSendThread for sending car's control orders has been destoried";
    m_isCanRun=true;
    if(!communication->connect()){
        return;
    }
    QString time = QDateTime::currentDateTime().toString("yyyyddMM_hhmmss");
    QFile file("./../QtControl/largeCarOrder/"+time+".txt");
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << QString("largeCarOrder.txt Open failed." );
        return;
    }else{
        qDebug() << QString("largeCarOrder.txt OpenSuccessful." );
    }
    QTextStream out(&file);
    while(true){ 
        if(!m_enable){
            current=&doNothingControlOrder;
        }else{
            current=&runControlOrder;
        }
        LargeCarCO largeCarCO;
        QMutexLocker locker1(&m_controlOrderLock);
        ControlOrder::NormalCO2LargeCarCO(*current,largeCarCO);
        locker1.unlock();
        unsigned char * c=largeCarCO.getCharOrder();
        for(int i=0;i<LARGECARCO_LENGTH;i++){
            out<<QString::number(c[i],16)<<" ";
        }
        QString localTime = QDateTime::currentDateTime().toString("ddMMyy hhmmss.zzz");
        out<<localTime<<"\n";
        communication->sendMessage(largeCarCO.getCharOrder());
        communication->receiveMessage();
        QMutexLocker locker2(&m_threadRunLock);
        if(!m_isCanRun)//在每次循环判断是否可以运行，如果不行就退出循环
        {
            locker2.unlock();
            break;
        }
        locker2.unlock();
        this->msleep(CONTROLORDERSENDTHREAD_BOLCKTIME);//阻塞否则cpu占用太高
    }
    file.close();
    communication->close();
}

void ControlOrderSendThread::setRange(int range){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setTurnRange(range);
    qDebug()<<"range change to "<<range;
    locker.unlock();
}

void ControlOrderSendThread::setSpeed(int speed){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setSpeed(speed);
    qDebug()<<"speed change to "<<speed;
    locker.unlock();
}

void ControlOrderSendThread::setGear(int gear){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setGear(gear);
    qDebug()<<"speed gear to "<<gear;
    locker.unlock();
}
