#ifndef CONTROLORDERSENDTOROSTHREAD_H
#define CONTROLORDERSENDTOROSTHREAD_H
#define CONTROLORDERSENDTOROSTHREAD_BOLCKTIME 50

#include "PreDefinition.h"
#include "LocationPosition.h"
#include "ultrasonic.h"
#include "RosPerceptionReceiveThread.h"
#include<CommunicationFactory.h>
#include<QThread>
#include<QMutex>
#include<QList>
#include<ControlOrder.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/QuaternionStamped.h"
/**
 * @brief 将开启一个线程，将汽车控制指令送往对应ROS
 */
class ControlOrderSendToRosThread : public QThread
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
    int m_perceptionDangerSignal;
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
    void sendTurnOrder(int ,ros::Publisher);
public:
    ControlOrderSendToRosThread(QObject *parent);
    ~ControlOrderSendToRosThread();
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

#endif // CONTROLORDERSENDTOROSTHREAD_H
