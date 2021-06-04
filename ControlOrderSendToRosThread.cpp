#include "ControlOrderSendToRosThread.h"
#include "GpsInfo.h"
#include <QDateTime>
#include <QObject>
ControlOrderSendToRosThread::ControlOrderSendToRosThread(QObject *parent):QThread(parent){
    readConfig();
    m_enable=false;
    m_doNothing=false;
    m_radarDangerSignal=false;
    m_perceptionDangerSignal=false;
    //通过槽函数绑定雷达数据
    radar=new Ultrasonic();
    // 设置配置文件名称
    radar->portConfigFileName = "ultrasonicPortNameConfig.txt"
    radar->openPort();
    connect(radar,&Ultrasonic::sendDistance,[this](bool flag,int distance){
        //qDebug()<<"radar distance:"<<distance;
        this->m_radarDangerSignal=flag;
    });

    //通过槽函数绑定感知
    perceptionReceiveThread=new RosPerceptionReceiveThread(this);
    connect(perceptionReceiveThread,&RosPerceptionReceiveThread::sendPerceptionSignal,[this](int flag){
//         if(flag)
//             qDebug()<<"perception detect danger!!";
         this->m_perceptionDangerSignal=flag;
    });
    perceptionReceiveThread->start();
}

ControlOrderSendToRosThread::~ControlOrderSendToRosThread(){
    //todo 关闭接口
    radar->closePort();
    perceptionReceiveThread->stopImmediately();
    perceptionReceiveThread->wait();

    qDebug()<<"ControlOrderSendToRosThread for sending car's control orders has been destoried";
}

void ControlOrderSendToRosThread::stopImmediately(){

    QMutexLocker locker(&m_threadRunLock);
    m_isCanRun = false;
    locker.unlock();
}
void ControlOrderSendToRosThread::enableSignal(bool signal){
    QMutexLocker locker(&m_enableLock);
    m_enable=signal;
    locker.unlock();
}

void ControlOrderSendToRosThread::run(){

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
    qDebug()<<"ControlOrderSendToRosThread for sending car's control orders started";
    m_isCanRun=true;
    //if(!communication->connect()){
        //return;  //todo
    //}

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
    int argc=0;
    char **argv={};
    ros::init(argc, argv, "order_send_node");
    ros::NodeHandle n("~");
    ros::Publisher order_simulate_pub = n.advertise<geometry_msgs::QuaternionStamped>("/tiny_car_order", 1000);
    ros::Rate loop_rate(20);
    //m_perceptionDangerSignal=PRECEPTION_LEFT;
    while(ros::ok()){
        if(m_enable&&!m_doNothing&&!m_radarDangerSignal&&m_perceptionDangerSignal==PRECEPTION_NOMAL){
            current=&runControlOrder;
        }else {
            if(!(m_enable&&!m_doNothing&&!m_radarDangerSignal)||m_perceptionDangerSignal==PRECEPTION_STOP){
                current=&doNothingControlOrder;
                qDebug()<<"do nothing           "
                       <<((m_doNothing)?"XYZ position is (0,0,0)":"XYZ position is nomal")
                       <<((m_radarDangerSignal)?"radar detect dange":"radar is nomal")
                       <<((m_perceptionDangerSignal==PRECEPTION_STOP)?"perception detect dange":"perception is nomal");
            }else if(m_perceptionDangerSignal==PRECEPTION_LEFT){
                //TODO
                qDebug()<<"AUTO DO RIGHT TURN";
                sendTurnOrder(PRECEPTION_LEFT,order_simulate_pub);
                m_perceptionDangerSignal=PRECEPTION_NOMAL;
                continue;
            }else if(m_perceptionDangerSignal==PRECEPTION_RIGHT){
                //TODO
                qDebug()<<"AUTO DO LEFT TURN";
                sendTurnOrder(PRECEPTION_LEFT,order_simulate_pub);
                m_perceptionDangerSignal=PRECEPTION_NOMAL;
                continue;
            }
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

        if(carType==LARGECARTYPE){
            //communication->sendMessage(largeCarCO.getCharOrder());
        }

        if(carType==TINYCARTYPE){
//            unsigned char tt[10]={0xFF,0xFE,1,2,3,4,5,6,7,8};
            //communication->sendMessage(tinyCarCO.getCharOrder());
            geometry_msgs::QuaternionStamped order;
            quint8* temp=tinyCarCO.getCharOrder();
            int leftSpeed=temp[3];
            int rightSpeed=temp[2];
            int leftOrientation=temp[5];
            int rightOrientation=temp[4];

            order.quaternion.w=leftOrientation;;
            order.quaternion.y=rightOrientation;
            order.quaternion.y=leftSpeed;
            order.quaternion.z=rightSpeed;

            order_simulate_pub.publish(order);
        }
        //for test
        //communication->receiveMessage();
        QMutexLocker locker2(&m_threadRunLock);
        if(!m_isCanRun)//在每次循环判断是否可以运行，如果不行就退出循环
        {
            //if disconnect ,setting car staying static
            if(carType==LARGECARTYPE)
                ControlOrder::NormalCO2LargeCarCO(doNothingControlOrder,largeCarCO);
                //communication->sendMessage(largeCarCO.getCharOrder());
            if(carType==TINYCARTYPE){
                ControlOrder::NormalCO2TinyCarCO(doNothingControlOrder,tinyCarCO);
                //communication->sendMessage(tinyCarCO.getCharOrder());
                geometry_msgs::QuaternionStamped order;
                quint8* temp=tinyCarCO.getCharOrder();
                int leftSpeed=temp[3];
                int rightSpeed=temp[2];
                int leftOrientation=temp[5];
                int rightOrientation=temp[4];

                order.quaternion.x=leftOrientation;;
                order.quaternion.w=rightOrientation;
                order.quaternion.y=leftSpeed;
                order.quaternion.z=rightSpeed;
                order_simulate_pub.publish(order);
            }
            qDebug()<<"order has been init to zero!";
            locker2.unlock();
            break;
        }
        locker2.unlock();
//        this->msleep(ControlOrderSendToRosThread_BOLCKTIME);//阻塞否则cpu占用太高
        ros::spinOnce();
        loop_rate.sleep();
    }
    file.close();
    file2.close();
    //communication->close();
    delete communication;
}

void ControlOrderSendToRosThread::setRange(int range){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setTurnRange(range);
    //qDebug()<<"range change to "<<range;
    locker.unlock();
}

void ControlOrderSendToRosThread::setSpeed(int speed){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setSpeed(speed);
    //qDebug()<<"speed change to "<<speed;
    locker.unlock();
}
void ControlOrderSendToRosThread::setGpsInfo(GpsInfo gpsInfo){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setGpsInfo(gpsInfo);
    doNothingControlOrder.setGpsInfo(gpsInfo);
    //qDebug()<<"gpsInfo change to "<<gpsInfo.toString();
    locker.unlock();
}
void ControlOrderSendToRosThread::setLocationInfo(LocationPosition location){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setLocation(location);
    doNothingControlOrder.setLocation(location);
    locker.unlock();
}
void ControlOrderSendToRosThread::setGear(int gear){
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setGear(gear);
    qDebug()<<"speed gear to "<<gear;
    locker.unlock();
}
void ControlOrderSendToRosThread::doNothing(bool signal){
    this->m_doNothing=signal;
}
void ControlOrderSendToRosThread::readConfig(){
    QFile file("./../QtControl/softwareConfig/config.txt");
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QByteArray t = file.readAll();
    QList<QByteArray>list =t.split(' ');
    systemType=list[0].toInt();
    carType=list[1].toInt();
    file.close();
}
void ControlOrderSendToRosThread::sendTurnOrder(int ordientation,ros::Publisher order_simulate_pub){
    int slow=8;
    int fast=32;
    int time=20*1000;
    int i=0;
    geometry_msgs::QuaternionStamped order;
    while(i<(time/CONTROLORDERSENDTOROSTHREAD_BOLCKTIME)){
        if(ordientation==PRECEPTION_LEFT){
            order.quaternion.y=fast;
            order.quaternion.z=slow;
        }
        order_simulate_pub.publish(order);
        i++;
        msleep(CONTROLORDERSENDTOROSTHREAD_BOLCKTIME);
    }
}
