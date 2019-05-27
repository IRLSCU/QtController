﻿#include "RosPerceptionReceiveThread.h"
#include <nav_msgs/Odometry.h>
#include <QDebug>

RosPerceptionReceiveThread::RosPerceptionReceiveThread(QObject *parent):QThread(parent){
}

RosPerceptionReceiveThread::~RosPerceptionReceiveThread(){
    qDebug()<<"RosReceiveThread for initing route has been destoried";
}

void RosPerceptionReceiveThread::stopImmediately(){
    QMutexLocker locker(&m_lock);
    m_isCanRun = false;
    locker.unlock();
    qDebug()<<"RosReceiveThread stopped";
}
void RosPerceptionReceiveThread::run(){
    qDebug()<<"RosReceiveThread for initing route has started";
    m_isCanRun=true;
    //ros::init(argc,argv,"listener");
    int argc=0;
    char **argv={};
    ros::init(argc, argv, "perception_receiver");
    ros::NodeHandle n("~");
    ros::Subscriber sub = n.subscribe(ROSPERCEPTIONRECEIVENODENAME, 1, &RosPerceptionReceiveThread::box_info_callback,this);
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
void RosPerceptionReceiveThread::box_info_callback(const fuse_all::distances::ConstPtr& msg)
{
    //double box_info_ts = msg->header.stamp.toSec();
    boxes_info.clear();
    dis.clear();
    bool danger=false;
    for(auto box : msg->distances){
        std::vector<float> tmp_vec(5);
        tmp_vec[0] = box.dis;
        tmp_vec[1] = box.xmin;
        tmp_vec[2] = box.xmax;
        tmp_vec[3] = box.ymin;
        tmp_vec[4] = box.ymax;
        dis.push_back(box.dis);
        //ROS_DEBUG("yolo_box_callback() dis: %lf, ", box.dis);
        boxes_info.push_back(tmp_vec);

        danger=danger||judgeDanger(box.dis,box.xmin,box.xmax,box.ymin,box.ymax);
    }
    emit sendPerceptionSignal(danger);
}
bool RosPerceptionReceiveThread::judgeDanger(float distance,float xmin,float xmax,float ymin,float ymax){
    if(distance<=EMERGYCEDISTANCE){
        qDebug()<<"something in emergeyce distance!!!!!!!!!";
        return true;
    }
    if(distance<=DANGEDISTANCE){
        qDebug()<<"something in dangerous distance!!!";
        return true;
    }
    return false;
    //todo 判断物体是否在车辆的正前方
}