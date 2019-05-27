#ifndef ROSPERCEPTIONRECEIVETHREAD_H
#define ROSPERCEPTIONRECEIVETHREAD_H
#define ROSPERCEPTIONRECEIVETHREAD_BOLCKTIME 50
#define ROSPERCEPTIONRECEIVENODENAME "/cur_pose"
#define DANGEDISTANCE 10
#define EMERGYCEDISTANCE 7
#define IMAGEWIDTH 752
#define IMAGEHEIGHT 480
#include "distance.h"
#include "distances.h"
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
class RosPerceptionReceiveThread : public QThread
{
    Q_OBJECT
private:
    QMutex m_lock;
    bool m_isCanRun;
    //string box_info_topic, distances_flag_topic;
    std::vector<std::vector<float >> boxes_info;
    std::vector<float > dis;
    bool judgeDanger(float,float,float,float,float);
public:
    RosPerceptionReceiveThread(QObject *parent=0);
    ~RosPerceptionReceiveThread();
    void stopImmediately();
    void run();
    void box_info_callback(const fuse_all::distances::ConstPtr& msg);
signals:
    void sendPerceptionSignal(bool);
};
#endif // ROSPERCEPTIONRECEIVETHREAD_H
