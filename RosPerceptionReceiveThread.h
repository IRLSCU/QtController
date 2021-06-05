#ifndef ROSPERCEPTIONRECEIVETHREAD_H
#define ROSPERCEPTIONRECEIVETHREAD_H


#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <QThread>
#include <QMutex>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

#include "RingBuffer.h"

#include "distance.h"
#include "distances.h"
/**
 * @brief ROS时间间隔函数
 */
#define ROSPERCEPTIONRECEIVETHREAD_BOLCKTIME 50
/**
 * @brief ros darknet 数据接收节点名称
 */
#define ROSPERCEPTIONRECEIVENODENAME "/darknet_ros/distance"
/**
 * @brief ros 单线激光雷达节点名称
 */
#define ROSSCANNAME "/scan"
/**
 * @brief 预定义危险距离
 */
#define DANGEDISTANCE 7
/**
 * @brief 单线激光雷达危险距离 \n
 * 即为ROS代价地图膨胀系数 \n
 */
#define LASERDANGERDISTANCE 0.01
/**
 * @brief 紧急制动距离
 */
#define EMERGYCEDISTANCE 4
/**
 * @brief 安全长度
 */
#define SAFELENGTH 2
/**
 * @brief 图像宽度
 */
#define IMAGEWIDTH 752
/**
 * @brief 图像高度
 */
#define IMAGEHEIGHT 480
/**
 * @brief 预测动作标准--静止
 */
#define PRECEPTION_NOMAL 0
/**
 * @brief 预测动作标准--停止
 */
#define PRECEPTION_STOP 1
/**
 * @brief 预测动作标准--左转
 */
#define PRECEPTION_LEFT 2
/**
 * @brief 预测动作标准--右转
 */
#define PRECEPTION_RIGHT 3

/**
 * @brief The RosReceiveThread \n 
 * 用于获得ROS中障碍物的信息之后进行处理,是简单粗糙的判断
 */
class RosPerceptionReceiveThread : public QThread
{
    Q_OBJECT

public:
    /**
     * @brief Construct a new RosPerceptionReceiveThread object
     * @param  parent           父类指针
     */
    RosPerceptionReceiveThread(QObject *parent=0);
    /**
     * @brief Destroy the RosPerceptionReceiveThread object
     */
    ~RosPerceptionReceiveThread();
    /**
     * @brief 紧急停止操作
     */
    void stopImmediately();
    /**
     * @brief 开始进行
     */
    void run();
    void box_info_callback(const darknet_ros_msgs::distances::ConstPtr& msg);
    void box_info_callback_laserscan(const sensor_msgs::LaserScan::ConstPtr& msg);
signals:
    void sendPerceptionSignal(int);

private:
    bool judgeDanger(float,float,float,float,float);
    bool judgeDangerByLaser(float x,float y,float z);
    QMutex m_lock;
    bool m_isCanRun;
    //string box_info_topic, distances_flag_topic;
    std::vector<std::vector<float >> boxes_info;
    std::vector<float > dis;
   
};
#endif // ROSPERCEPTIONRECEIVETHREAD_H
