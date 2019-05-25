﻿#include "ControlOrderSendThread.h"
#include "GpsInfo.h"
#include <QDateTime>
#include <QObject>
ControlOrderSendThread::ControlOrderSendThread(QObject *parent=0):QThread(parent){
    readConfig();
    m_enable=false;
    m_doNothing=false;
    m_radarDangerSignal=false;
    radar=new Ultrasonic;
    radar->openPort();
    connect(radar,&Ultrasonic::sendDistance,[this](bool flag,int distance){
        //qDebug()<<"radar distance:"<<distance;
        this->m_radarDangerSignal=flag;
    });
}

ControlOrderSendThread::~ControlOrderSendThread(){
    //todo 关闭接口
    radar->closePort();
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

    //初始化接口
    if(carType==LARGECARTYPE){
        if(systemType==PREDEFINITIONWINDOWS){
            communicationType=LargeCarWindows;
        }else if(systemType==PREDEFINITIONLINUX){
            communicationType=LargeCarLinux;
        }
    }else if(carType==TINYCARTYPE){
        communicationType=TinyCarLinux;
    }
    communication=CommunicationFactory::createCommunication(communicationType);
    //todo 打开接口
    qDebug()<<"ControlOrderSendThread for sending car's control orders started";
    m_isCanRun=true;
    if(!communication->connect()){
        //return;  todo
    }

//    unsigned char tt[10]={0xFF,0xFE,1,2,3,4,5,6,7,8};
//    communication->sendMessage(tt);

    QString time = QDateTime::currentDateTime().toString("yyyyddMM_hhmmss");
    QFile file("./../QtControl/largeCarOrder/GPS"+time+".txt");
    QFile file2("./../QtControl/largeCarOrder/XYZ"+time+".txt");
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << QString("GPSlargeCarOrder.txt Open failed." );
        return;
    }else{
        qDebug() << QString("GPSlargeCarOrder.txt OpenSuccessful." );
    }

    if(!file2.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << QString("XYZlargeCarOrder.txt Open failed." );
        return;
    }else{
        qDebug() << QString("XYZlargeCarOrder.txt OpenSuccessful." );
    }

    QTextStream out(&file);
    QTextStream out2(&file2);
    while(true){ 
        if(m_enable&&!m_doNothing&&!m_radarDangerSignal){
            current=&runControlOrder;
            //qDebug()<<"do it";
        }else{
            current=&doNothingControlOrder;
            //qDebug()<<"do nothing";
        }
        LargeCarCO largeCarCO;
        TinyCarCO tinyCarCO;
        QMutexLocker locker1(&m_controlOrderLock);
        ControlOrder::NormalCO2LargeCarCO(*current,largeCarCO);
        ControlOrder::NormalCO2TinyCarCO(*current,tinyCarCO);
        locker1.unlock();
        //tinyCarCO.printInfo();
        //current->printInfo();
        unsigned char * c=largeCarCO.getCharOrder();
        out<<current->getGpsInfo().toString()<<" ";
        out2<<current->getLocation().toString()<<" ";

        for(int i=0;i<LARGECARCO_LENGTH;i++){
            out<<QString::number(c[i],10)<<" ";
            out2<<QString::number(c[i],10)<<" ";
        }
        QString localTime = QDateTime::currentDateTime().toString("ddMMyy hhmmss.zzz");
        out<<localTime<<"\n";
        out2<<localTime<<"\n";

        //todo
        if(carType==LARGECARTYPE)
//            largeCarCO.setSpeed(107);
            communication->sendMessage(largeCarCO.getCharOrder());
        if(carType==TINYCARTYPE){
//            unsigned char tt[10]={0xFF,0xFE,1,2,3,4,5,6,7,8};
            communication->sendMessage(tinyCarCO.getCharOrder());
        }
        //for test
        //communication->receiveMessage();
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
    file2.close();
    communication->close();
    delete communication;
}

void ControlOrderSendThread::setRange(int range){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setTurnRange(range);
    //qDebug()<<"range change to "<<range;
    locker.unlock();
}

void ControlOrderSendThread::setSpeed(int speed){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setSpeed(speed);
    //qDebug()<<"speed change to "<<speed;
    locker.unlock();
}
void ControlOrderSendThread::setGpsInfo(GpsInfo gpsInfo){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setGpsInfo(gpsInfo);
    //qDebug()<<"gpsInfo change to "<<gpsInfo.toString();
    locker.unlock();
}
void ControlOrderSendThread::setLocationInfo(LocationPosition location){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setLocation(location);
    locker.unlock();
}
void ControlOrderSendThread::setGear(int gear){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setGear(gear);
    qDebug()<<"speed gear to "<<gear;
    locker.unlock();
}
void ControlOrderSendThread::doNothing(bool signal){
    this->m_doNothing=signal;
}
void ControlOrderSendThread::readConfig(){
    QFile file("./../QtControl/softwareConfig/config.txt");
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QByteArray t = file.readAll();
    QList<QByteArray>list =t.split(' ');
    systemType=list[0].toInt();
    carType=list[1].toInt();
    file.close();
}
