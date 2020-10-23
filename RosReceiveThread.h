#ifndef ROSRECEIVETHREAD_H
#define ROSRECEIVETHREAD_H
#define ROSRECEIVETHREAD_BOLCKTIME 50
#define ROSRADAR
#ifdef ROSRADAR  //接收单线激光雷达传的定位信息
#define ROSRECEIVENODENAME "/tf"
#else //接收双目视觉定位信息
#define ROSRECEIVENODENAME "/cur_pose"
#endif
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include <nav_msgs/Odometry.h>

#include "RingBuffer.h"
#include <sstream>
#include<QThread>
#include<QMutex>

/**
 * @brief The RosReceiveThread
 * 接收ROS传过来的定位信息
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
#ifdef ROSRADAR
    void chatterCallback(const tf2_msgs::TFMessage::ConstPtr tfmsg);
#else
    void chatterCallback(const geometry_msgs::Pose xyz);
#endif
signals:
    void sendMessage(QString);
};
#endif // ROSRECEIVETHREAD_H
