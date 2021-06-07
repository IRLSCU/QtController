/**
 * @file GpsBufferConsumeRunThread.h
 * @brief GPS读取缓冲线程类GpsBufferConsumeRunThread 头文件
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-07 18:50:29
 * @copyright Copyright (c) 2021  IRLSCU
 * 
 * @par 修改日志:
 * <table>
 * <tr>
 *    <th> Commit date</th>
 *    <th> Version </th> 
 *    <th> Author </th>  
 *    <th> Description </th>
 * </tr>
 * <tr>
 *    <td> 2021-06-07 18:50:29 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加注释文档 </td>
 * </tr>
 * </table>
 */
#ifndef GPSBUFFERCONSUMERUNTHREAD_H
#define GPSBUFFERCONSUMERUNTHREAD_H

#include <QThread>
#include <QMutex>
#include <QList>

#include "RingBuffer.h"
#include "GPSProcesser.h"
#include "PID.h"
/**
 * @brief GPS指令发送时间间隔
 */
#define GPSBUFFERCONSUMERUNTHREAD_BOLCKTIME 50

/**
 * @brief The GpsBufferConsumeRunThread class
 * consume gpsBuffer to init route thread
 * 消费者线程、开始消费GpsInfo信息
 */
class GpsBufferConsumeRunThread : public QThread
{
    Q_OBJECT

public:
    /**
     * @brief Construct a new GpsBufferConsumeRunThread object
     * @param  gpsRingBuffer    gps环形缓冲区
     * @param  parent           父类指针
     */
    GpsBufferConsumeRunThread(GpsRingBuffer *gpsRingBuffer, QObject *parent);
    /**
     * @brief Destroy the GpsBufferConsumeRunThread object
     */
    ~GpsBufferConsumeRunThread();
    /**
     * @brief 立即停止
     */
    void stopImmediately();
    /**
     * @brief  设置是否开启信号值
     * @param  signal           My Param doc
     */
    void runSignal(bool signal);
    /**
     * @brief 发送起始坐标给ProcessRunDialog开关打开
     */
    void setSendStartGpsInfoSignal();
    void run();

signals:
    /**
     * @brief 每接收一个点就发送，发送给paintWeidge绘制
     * @param  point            GPS坐标点
     */
    void sendGpsInfo(QPointF point);
    /**
     * @brief  当开始初始化路径时(m_startInit=true)，一直发送gps信息
     * @param  point            GPS坐标点
     */
    void sendRunGpsInfo(GpsInfo point);
    /**
     * @brief m_isSendStartGpsInfo=true,每次只发送一次后，m_isSendStartGpsInfo=false
     * @param  point            GPS坐标点
     */
    void sendStartGpsInfo(GpsInfo point);

private:
    GpsRingBuffer *gpsRingBuffer;  ///< gps环形缓冲区
    QMutex m_lock;                 ///< 线程运行锁信号量
    QMutex m_Initlock;             ///< 初始化锁信号量，用来保护m_startInit变量
    QMutex m_sendStartGpsInfoLock; ///< m_isSendStartGpsInfo 保护锁信号量
    bool m_isCanRun;               ///< 控制线程开闭
    bool m_startInit;              ///< 是否开始RUN
    bool m_isSendStartGpsInfo;     ///< 是否发送GPS信息
};

#endif // GPSBUFFERCONSUMERUNTHREAD_H
