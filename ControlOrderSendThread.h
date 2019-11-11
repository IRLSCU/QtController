#ifndef CONTROLORDERSENDTHREAD_H
#define CONTROLORDERSENDTHREAD_H
#define CONTROLORDERSENDTHREAD_BOLCKTIME 50

#include "PreDefinition.h"
#include "LocationPosition.h"
#include "ultrasonic.h"
#include "RosPerceptionReceiveThread.h"
#include<CommunicationFactory.h>
#include<QThread>
#include<QMutex>
#include<QList>
#include<ControlOrder.h>

/**
 * @brief 将开启一个线程，将汽车控制指令送往对应车辆
 */
class ControlOrderSendThread : public QThread
{
    Q_OBJECT
private:
    QMutex m_threadRunLock;
    QMutex m_enableLock;
    QMutex m_controlOrderLock;
    bool m_isCanRun;//控制线程开闭
    bool m_enable;//是否开始RUN
    bool m_doNothing;//未接受到正确的数据(x=y=z=0)
    bool m_radarDangerSignal;//超声波雷达监测到前方某一距离有障碍物 true=stop;false=run
    int m_perceptionDangerSignal;//融合感知模块传递障碍物
    ControlOrder doNothingControlOrder;
    ControlOrder runControlOrder;
    ControlOrder *current;
    CommunicationType communicationType;
    AbstractCommunication* communication;
    Ultrasonic* radar;
    RosPerceptionReceiveThread* perceptionReceiveThread;
    //TinyCarCommunication* communication;
    int systemType;
    int carType;//
    void readConfig();
public:
    ControlOrderSendThread(QObject *parent);
    ~ControlOrderSendThread();
    void stopImmediately();
    void enableSignal(bool signal);
    void doNothing(bool signal);
    void run();
    void setRange(int range);
    void setSpeed(int speed);
    void setGpsInfo(GpsInfo gps);
    void setLocationInfo(LocationPosition location);
    void setGear(int gear);
signals:
};

#endif // CONTROLORDERSENDTHREAD_H
