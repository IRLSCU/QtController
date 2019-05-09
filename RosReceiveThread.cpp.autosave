#include "RosReceiveThread.h"
#include <nav_msgs/Odometry.h>
#include <QDebug>

RosReceiveThread::RosReceiveThread(LocationRingBuffer* locationRingBuffer,QObject *parent):QThread(parent){
    this->locationRingBuffer=locationRingBuffer;
}

RosReceiveThread::~RosReceiveThread(){
    qDebug()<<"RosReceiveThread for initing route has been destoried";
}

void RosReceiveThread::stopImmediately(){
    QMutexLocker locker(&m_lock);
    m_isCanRun = false;
    locker.unlock();
    qDebug()<<"RosReceiveThread stopped";
}
void RosReceiveThread::run(){
    qDebug()<<"RosReceiveThread for initing route has started";
    m_isCanRun=true;
    //ros::init(argc,argv,"listener");
    int argc=0;
    char **argv={};
    ros::init(argc, argv, "ControlListener");
    ros::NodeHandle n("~");
    ros::Subscriber sub = n.subscribe(ROSRECEIVENODENAME, 1, &RosReceiveThread::chatterCallback,this);
    ros::Rate loop_rate(20);
    int count = 0;
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
//        qDebug()<<count;
        QMutexLocker locker(&m_lock);
        if(!m_isCanRun)//在每次循环判断是否可以运行，如果不行就退出循环
        {
            return;
        }else{
            locker.unlock();
        }
    }
}
void RosReceiveThread::chatterCallback(const geometry_msgs::Pose xyz){
    //ROS_INFO("I heard: [%f][%f][%f]", xyz.position.x,xyz.position.y,xyz.position.z);
    //qDebug()<<"I heard"<<xyz.position.x<<xyz.position.y<<xyz.position.z;
    LocationPosition position(xyz.position.x,xyz.position.z,xyz.position.y);
    locationRingBuffer->push(position);
    sendMessage(position.toString().append("\n"));
}
