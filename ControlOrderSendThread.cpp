#include <QDateTime>
#include <QObject>

#include "ControlOrderSendThread.h"
#include "GpsInfo.h"

ControlOrderSendThread::ControlOrderSendThread(QObject *parent) : QThread(parent)
{
    // 读取配置文件
    readConfig();

    m_enable = false;
    m_doNothing = false;
    m_radarDangerSignal = false;
    m_perceptionDangerSignal = false;
    // 设置超声波雷达
    radar = new Ultrasonic(this);
    // 毫米波雷达默认配置文件
    radar->portConfigFileName = "ultrasonicPortNameConfig.txt";
    radar->openPort();
    //通过槽函数绑定雷达数据
    connect(radar, &Ultrasonic::sendDistance, [this](bool flag, int distance)
            {
                //qDebug()<<"radar distance:"<<distance;
                this->m_radarDangerSignal = flag;
            });

    // 通过槽函数绑定感知
    perceptionReceiveThread = new RosPerceptionReceiveThread(this);
    connect(perceptionReceiveThread, &RosPerceptionReceiveThread::sendPerceptionSignal, [this](int flag)
            {
                //         if(flag)
                //             qDebug()<<"perception detect danger!!";
                this->m_perceptionDangerSignal = flag;
            });
    perceptionReceiveThread->start();
}

ControlOrderSendThread::~ControlOrderSendThread()
{
    //todo 关闭接口
    radar->closePort();
    perceptionReceiveThread->stopImmediately();
    perceptionReceiveThread->wait();

    qDebug() << "ControlOrderSendThread for sending car's control orders has been destoried";
}

void ControlOrderSendThread::stopImmediately()
{

    QMutexLocker locker(&m_threadRunLock);
    m_isCanRun = false;
    locker.unlock();
}
void ControlOrderSendThread::enableSignal(bool signal)
{
    QMutexLocker locker(&m_enableLock);
    m_enable = signal;
    locker.unlock();
}

void ControlOrderSendThread::run()
{

    //初始化接口
    if (carType == LARGECARTYPE)
    {
        if (systemType == PREDEFINITIONWINDOWS)
        {
            communicationType = LargeCarWindows;
        }
        else if (systemType == PREDEFINITIONLINUX)
        {
            communicationType = LargeCarLinux;
        }
    }
    else if (carType == TINYCARTYPE)
    {
        communicationType = TinyCarLinux;
    }
    // 策略工厂创建交流类
    communication = CommunicationFactory::createCommunication(communicationType);
    //todo 打开接口
    qDebug() << "ControlOrderSendThread for sending car's control orders started";
    m_isCanRun = true;
    if (!communication->connect())
    {
        //return;  //todo
        qDebug() << "connect error"
    }

    //    unsigned char tt[10]={0xFF,0xFE,1,2,3,4,5,6,7,8};
    //    communication->sendMessage(tt);

    QString time = QDateTime::currentDateTime().toString("yyyyddMM_hhmmss");
    /**
     * @todo 改性相关数据存储位置
     */
    QFile file("largeCarOrder/GPS" + time + ".txt");
    QFile file2("largeCarOrder/XYZ" + time + ".txt");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << QString("GPSlargeCarOrder.txt Open failed.");
        return;
    }
    else
    {
        qDebug() << QString("GPSlargeCarOrder.txt OpenSuccessful.");
    }

    if (!file2.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << QString("XYZlargeCarOrder.txt Open failed.");
        return;
    }
    else
    {
        qDebug() << QString("XYZlargeCarOrder.txt OpenSuccessful.");
    }

    QTextStream out(&file);
    QTextStream out2(&file2);
    while (true)
    {
        if (m_enable && !m_doNothing && !m_radarDangerSignal && m_perceptionDangerSignal == PRECEPTION_NOMAL)
        {
            current = &runControlOrder;
        }
        else
        {
            if (!(m_enable && !m_doNothing && !m_radarDangerSignal) || m_perceptionDangerSignal == PRECEPTION_STOP)
            {
                current = &doNothingControlOrder;
                qDebug() << "do nothing           "
                         << ((m_doNothing) ? "XYZ position is (0,0,0)" : "XYZ position is nomal")
                         << ((m_radarDangerSignal) ? "radar detect dange" : "radar is nomal")
                         << ((m_perceptionDangerSignal == PRECEPTION_STOP) ? "perception detect dange" : "perception is nomal");
            }
            else if (m_perceptionDangerSignal == PRECEPTION_LEFT)
            {
                //TODO
                qDebug() << "AUTO DO RIGHT TURN";
            }
            else if (m_perceptionDangerSignal == PRECEPTION_RIGHT)
            {
                //TODO
                qDebug() << "AUTO DO LEFT TURN";
            }
        }
        LargeCarCO largeCarCO;
        TinyCarCO tinyCarCO;
        QMutexLocker locker1(&m_controlOrderLock);
        ControlOrder::NormalCO2LargeCarCO(*current, largeCarCO);
        ControlOrder::NormalCO2TinyCarCO(*current, tinyCarCO);
        locker1.unlock();
        //tinyCarCO.printInfo();
        //current->printInfo();
        unsigned char *c = largeCarCO.getCharOrder();
        out << current->getGpsInfo().toString() << " ";
        out2 << current->getLocation().toString() << " ";
        for (int i = 0; i < LARGECARCO_LENGTH; i++)
        {
            out << QString::number(c[i], 10) << " ";
            out2 << QString::number(c[i], 10) << " ";
        }
        QString localTime = QDateTime::currentDateTime().toString("ddMMyy hhmmss.zzz");
        out << localTime << "\n";
        out2 << localTime << "\n";

        if (carType == LARGECARTYPE)
        {
            communication->sendMessage(largeCarCO.getCharOrder());
        }

        if (carType == TINYCARTYPE)
        {
            //            unsigned char tt[10]={0xFF,0xFE,1,2,3,4,5,6,7,8};
            communication->sendMessage(tinyCarCO.getCharOrder());
        }
        //for test
        //communication->receiveMessage();
        QMutexLocker locker2(&m_threadRunLock);
        if (!m_isCanRun) //在每次循环判断是否可以运行，如果不行就退出循环
        {
            //if disconnect ,setting car staying static
            if (carType == LARGECARTYPE)
                ControlOrder::NormalCO2LargeCarCO(doNothingControlOrder, largeCarCO);
            communication->sendMessage(largeCarCO.getCharOrder());
            if (carType == TINYCARTYPE)
            {
                ControlOrder::NormalCO2TinyCarCO(doNothingControlOrder, tinyCarCO);
                communication->sendMessage(tinyCarCO.getCharOrder());
            }
            qDebug() << "order has been init to zero!";
            locker2.unlock();
            break;
        }
        locker2.unlock();
        /**
         * @brief 发送频率过低可能存在问题
         * @bug 阻塞否则cpu占用太高
         * @todo 进行修复
         */
        this->msleep(CONTROLORDERSENDTHREAD_BOLCKTIME);
    }
    file.close();
    file2.close();
    communication->close();
    delete communication;
}

void ControlOrderSendThread::setRange(int range)
{
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setTurnRange(range);
    //qDebug()<<"range change to "<<range;
    locker.unlock();
}

void ControlOrderSendThread::setSpeed(int speed)
{
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setSpeed(speed);
    //qDebug()<<"speed change to "<<speed;
    locker.unlock();
}
void ControlOrderSendThread::setGpsInfo(GpsInfo gpsInfo)
{
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setGpsInfo(gpsInfo);
    doNothingControlOrder.setGpsInfo(gpsInfo);
    //qDebug()<<"gpsInfo change to "<<gpsInfo.toString();
    locker.unlock();
}
void ControlOrderSendThread::setLocationInfo(LocationPosition location)
{
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setLocation(location);
    doNothingControlOrder.setLocation(location);
    locker.unlock();
}
void ControlOrderSendThread::setGear(int gear)
{
    QMutexLocker locker(&m_controlOrderLock);
    runControlOrder.setGear(gear);
    qDebug() << "speed gear to " << gear;
    locker.unlock();
}
void ControlOrderSendThread::doNothing(bool signal)
{
    this->m_doNothing = signal;
}
void ControlOrderSendThread::readConfig()
{
    QFile file(m_orderConfigFilePath);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QByteArray t = file.readAll();
    QList<QByteArray> list = t.split(' ');
    systemType = list[0].toInt();
    carType = list[1].toInt();
    file.close();
}
