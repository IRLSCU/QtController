#ifndef GPSBUFFERREADINITROUTETHREAD_H
#define GPSBUFFERREADINITROUTETHREAD_H

#include <QThread>
#include <QMutex>
#include <QList>

#include "RingBuffer.h"

#define GPSBUFFERREADINITROUTETHREAD_BOLCKTIME 50

/**
 * @brief The GpsBufferReadInitRouteThread class
 * consume gpsBuffer to init route thread
 * 消费者线程去初始化路径
 */
class GpsBufferReadInitRouteThread : public QThread
{
    Q_OBJECT
public:
    /**
     * @brief Construct a new GpsBufferReadInitRoute Thread object
     * @param  gpsRingBuffer    GPS环形缓冲区
     * @param  parent           父类指针
     */
    GpsBufferReadInitRouteThread(GpsRingBuffer *gpsRingBuffer, QObject *parent = 0);
    /**
     * @brief Destroy the Gps BufferReadInitRouteThread object
     */
    ~GpsBufferReadInitRouteThread();
    /**
     * @brief 停止发送指令
     */
    void stopImmediately();
    /**
     * @brief  初始化线程信号
     * @param  signal           相关信号量
     */
    void initSignal(bool signal);
    /**
     * @brief 快速可执行路径
     */
    void run();

signals:
    /**
     * @brief 发送给paintWeidge绘制
     * @param  point            GPS相对位置坐标点
     */
    void sendGpsInfo(QPointF point);
    /**
     * @brief 开始初始化路径时，发送gps信息
     * @param  point            初始化位置坐标点
     */
    void sendInitGpsInfo(GpsInfo point);

private:
    /**
     * @brief  GPS数据缓冲buffer
     */
    GpsRingBuffer *gpsRingBuffer;
    /**
     * @brief  指令发送数据保护锁
     */
    QMutex m_lock;
    /**
     * @brief 运行线程开启保护锁信号量
     */
    QMutex m_Initlock;
    /**
     * @brief  是否开始发送控制指令
     */
    bool m_isCanRun;
    /**
     * @brief 是否开始初始化
     */
    bool m_startInit;

};

#endif // GPSBUFFERREADINITROUTETHREAD_H
