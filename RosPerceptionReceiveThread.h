#ifndef ROSPERCEPTIONRECEIVETHREAD_H
#define ROSPERCEPTIONRECEIVETHREAD_H
#define ROSPERCEPTIONRECEIVETHREAD_BOLCKTIME 50
#define ROSPERCEPTIONRECEIVENODENAME "/darknet_ros/distance"
#define ROSSCANNAME "/scan"
#define DANGEDISTANCE 7
#define LASERDANGERDISTANCE 0.01
#define EMERGYCEDISTANCE 4
#define SAFELENGTH 2
#define IMAGEWIDTH 752
#define IMAGEHEIGHT 480
#include "distance.h"
#include "distances.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
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
    bool judgeDangerByLaser(float x,float y,float z);
public:
    RosPerceptionReceiveThread(QObject *parent=0);
    ~RosPerceptionReceiveThread();
    void stopImmediately();
    void run();
    void box_info_callback(const darknet_ros_msgs::distances::ConstPtr& msg);
    void box_info_callback_laserscan(const sensor_msgs::LaserScan::ConstPtr& msg);
signals:
    void sendPerceptionSignal(bool);
};
#endif // ROSPERCEPTIONRECEIVETHREAD_H
