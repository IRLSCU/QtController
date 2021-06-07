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
    RosPerceptionReceiveThread(QObject *parent = 0);
    /**
     * @brief Destroy the RosPerceptionReceiveThread object
     */
    ~RosPerceptionReceiveThread();
    /**
     * @brief 紧急停止操作
     */
    void stopImmediately();
    /**
     * @brief 开始进行，主要是使用ROS节点进行消息订阅
     */
    void run();
    /**
     * @brief  darknet_ros感知紧急制动判断
     * @bug 紧急制动信号抛出异常
     * @param  msg             
     */
    void box_info_callback(const darknet_ros_msgs::distances::ConstPtr &msg);
    /**
     * @brief  通过单线激光雷达扫描信息进行小车的紧急制动判断
     * @param  msg              小车紧急制动判断
     */
    void box_info_callback_laserscan(const sensor_msgs::LaserScan::ConstPtr &msg);
signals:
    /**
     * @brief  发送预判断主要信号
     * @param  sendSingnal      主要信号
     */
    void sendPerceptionSignal(int sendSingnal);

private:
    /**
     * @brief  判断是否在紧急距离或者危险距离，并输出日志信息
     * @param  distance         My Param doc
     * @param  xmin             My Param doc
     * @param  xmax             My Param doc
     * @param  ymin             My Param doc
     * @param  ymax             My Param doc
     * @return true 
     * @return false 
     */
    bool judgeDanger(float distance, float xmin, float xmax, float ymin, float ymax);
    /**
     * @brief  是否在雷达范围内发生紧急制动
     * @param  x                My Param doc
     * @param  y                My Param doc
     * @param  z                My Param doc
     * @return true 
     * @return false 
     */
    bool judgeDangerByLaser(float x, float y, float z);
    QMutex m_lock;                              ///< 线程所
    bool m_isCanRun;                            ///< USB控制接口是否正在正常运行
    std::vector<std::vector<float>> boxes_info; ///< dark_net接收到的目标信息
    std::vector<float> dis;                     ///< 对应文本框的目标检测距离函数
};
#endif // ROSPERCEPTIONRECEIVETHREAD_H
