#ifndef CONTROLORDERSENDTHREAD_H
#define CONTROLORDERSENDTHREAD_H
#define CONTROLORDERSENDTHREAD_BOLCKTIME 200

#include "PreDefinition.h"
#include "LocationPosition.h"
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
    ControlOrder doNothingControlOrder;
    ControlOrder runControlOrder;
    ControlOrder *current;
    CommunicationType communicationType;
    AbstractCommunication* communication;
    //TinyCarCommunication* communication;
    int systemType;
    int carType;//
    void readConfig();
public:
    ControlOrderSendThread(QObject *parent);
    ~ControlOrderSendThread();
    void stopImmediately();
    void enableSignal(bool signal);
    void run();
    void setRange(int range);
    void setSpeed(int speed);
    void setGpsInfo(GpsInfo gps);
    void setLocationInfo(LocationPosition location);
    void setGear(int gear);
signals:
};

#endif // CONTROLORDERSENDTHREAD_H
