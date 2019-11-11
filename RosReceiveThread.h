#ifndef ROSRECEIVETHREAD_H
#define ROSRECEIVETHREAD_H
#define ROSRECEIVETHREAD_BOLCKTIME 50
#define ROSRECEIVENODENAME "/cur_pose"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <nav_msgs/Odometry.h>

#include "RingBuffer.h"
#include <sstream>
#include<QThread>
#include<QMutex>

/**
 * @brief The RosReceiveThread
 */
class RosReceiveThread : public QThread
{
    Q_OBJECT
private:
    QMutex m_lock;
    bool m_isCanRun;
    LocationRingBuffer* locationRingBuffer;
public:
    RosReceiveThread(LocationRingBuffer*,QObject *parent=0);
    ~RosReceiveThread();
    void stopImmediately();
    void run();
    void chatterCallback(const geometry_msgs::Pose xyz);
signals:
    void sendMessage(QString);
};
#endif // ROSRECEIVETHREAD_H
