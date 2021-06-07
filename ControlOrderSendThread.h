/**
 * @file ControlOrderSendThread.h
 * @brief 控制命令发送线程类ControlOrderSendThread主要头文件
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-05 16:45:17
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
 *    <td> 2021-06-05 16:45:17 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加文档注释 </td>
 * </tr>
 * </table>
 */
#ifndef CONTROLORDERSENDTHREAD_H
#define CONTROLORDERSENDTHREAD_H

#include <CommunicationFactory.h>
#include <QThread>
#include <QMutex>
#include <QList>
#include <ControlOrder.h>

#include "PreDefinition.h"
#include "LocationPosition.h"
#include "ultrasonic.h"
#include "RosPerceptionReceiveThread.h"
/**
 * @brief 指令发送线程阻塞等待时间
 */
#define CONTROLORDERSENDTHREAD_BOLCKTIME 50

/**
 * @brief 将开启一个线程，将汽车控制指令送往对应车辆串口 \n
 * @details \n 
 * ControlOrderSendToRosThread与ControlOrderSendThread二选一, \n 
 * ControlOrderSendToRosThread通过ROS节点将数据发往小车 \n 
 * ControlOrderSendThread直接通过串口发往小车.在ProcessRunNoGPSDialog中用宏定义选择 \n 
 * @todo 使用原子操作代替部分锁
 */
class ControlOrderSendThread : public QThread
{
    Q_OBJECT

public:
    /**
     * @brief Construct a new ControlOrderSendThread object
     * @param  parent           
     */
    ControlOrderSendThread(QObject *parent);
    /**
     * @brief Destroy the ControlOrderSendThread object
     */
    ~ControlOrderSendThread();
    /**
     * @brief 立刻停止
     */
    void stopImmediately();
    /**
     * @brief 允许信号发送
     * @param  signal           是否发送信号
     */
    void enableSignal(bool signal);
    /**
     * @brief  设置doNothing 值
     * @param  signal           是否接收到正确数值
     */
    void doNothing(bool signal);
    /**
     * @brief 程序运行
     */
    void run();
    /**
     * @brief  设置转向
     * @param  range            转动角度
     */
    void setRange(int range);
    /**
     * @brief 设置速度
     * @param  speed            速度值
     */
    void setSpeed(int speed);
    /**
     * @brief 设置中心GPS坐标值
     * @param  gps              GPS值
     */
    void setGpsInfo(GpsInfo gps);
    /**
     * @brief 设置相对位置中心值
     * @param  location         相对位置中心
     */
    void setLocationInfo(LocationPosition location);
    /**
     * @brief 设置档位
     * @details 详细信息见: \n
     * @see LargeCarCO
     * @param  gear             档位值
     */
    void setGear(int gear);
    /**
     * @brief Set the ConfigFilePath object
     * @param  new_file_path    新配置文件地址
     */
    inline void setConfigFilePath(QString new_file_path) { m_orderConfigFilePath = new_file_path; }
    /**
     * @brief Get the ConfigFilePath object
     * @return QString 获取配置文件地址
     */
    inline QString getConfigFilePath() { return m_orderConfigFilePath; }

private:
    /**
     * @brief 配置文件读取函数
     */
    void readConfig();

    QMutex m_threadRunLock;                                      ///< m_isCanRun数据读取线程保护信号量
    QMutex m_enableLock;                                         ///< m_enable 数据保护锁信号量
    QMutex m_controlOrderLock;                                   ///< runControlOrder 数据读取保护信号量
    bool m_isCanRun;                                             ///< 控制线程开闭
    bool m_enable;                                               ///< 是否开始RUN
    bool m_doNothing;                                            ///< 未接受到正确的数据(x=y=z=0)
    bool m_radarDangerSignal;                                    ///< 超声波雷达监测到前方某一距离有障碍物 true=stop;false=run
    int m_perceptionDangerSignal;                                ///< 融合感知模块传递障碍物
    ControlOrder doNothingControlOrder;                          ///< 非运动状态下(没有开始运动时)控制指令，配合发送指令按钮决定是运行还是非运行指令
    ControlOrder runControlOrder;                                ///< 运动控制指令
    ControlOrder *current;                                       ///< 当前的指令抽象
    CommunicationType communicationType;                         ///< 指令发送类型，由此通过格式工厂进行对应控制类的创建
    AbstractCommunication *communication;                        ///< 最终生成的指令创建类
    Ultrasonic *radar;                                           ///< 超声波雷达指针
    RosPerceptionReceiveThread *perceptionReceiveThread;         ///< 障碍物距离接收的判断线程
    QString m_orderConfigFilePath = "softwareConfig/config.txt"; ///< 配置文件路径
    int systemType;                                              ///< 系统类型
    int carType;                                                 ///< CAN 口类型(大车还是小车)
};

#endif // CONTROLORDERSENDTHREAD_H
