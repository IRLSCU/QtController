/**
 * @file ultrasonic.h
 * @brief 超声波雷达测距方式类
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-04 15:46:26
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
 *    <td> 2021-06-04 15:46:26 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加文档注释 </td>
 * </tr>
 * </table>
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <QDebug>
#include <QSerialPort>
#include <QObject>
#include <QString>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <QLatin1Char>
#include <QLatin1String>

#include "fileoperation.h"
/**
 * @brief 小车停止的门限距离
 */
#define THRESHOLD 50

/**
 * @brief 超声波雷达测距管理类
 * @details 超声波雷达,用于超声波测距,小于某一个阈值则刹车 \n
 * 详细工作原理请参考：
 * - [GY-US42V1使用说明](https://github.com/IRLSCU/documentation_for_devices/blob/26c5b3dea9700df4b621c363b03d22ba311dca17/GY_US42/GY-US42V1%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8Ev1.01.pdf)
 * 
 */
class Ultrasonic : public QObject
{
    Q_OBJECT
public:
    /**
     * @brief Construct a new Ultrasonic object
     * @param  parent           Qt parent
     */
    explicit Ultrasonic(QObject *parent = nullptr);
    /**
     * @brief Destroy the Ultrasonic object
     */
    ~Ultrasonic();
    /**
     * @brief 打开串口
     */
    void openPort();
    /**
     * @brief 关闭串口
     */
    void closePort();
    /**
     * @brief Get the Distance object
     * @return int 检测距离值(cm)
     */
    int getDistance();
    /**
     * @brief  将读取到的
     * @param  s              待处理原始16进制字符串  
     * @return int            最终输出结果 
     */
    int hexString2Int(QString s);
    /**
     * @brief 对读取数据进行校验。
     * @details 校验成功数据为真实数据，才能进行下一步操作
     * @param  s                读取到的原始字符串
     * @return true             数据校验成功
     * @return false            数据校验失败
     */
    bool checkData(QString s);
public slots:
    /**
     * @brief  Qt槽函数，用于读取设备文件
     * @return QString 串口读取到的16进制数据字符串
     */
    QString receiveInfo();
    /**
     * @brief  Qt槽函数，计算真实距离
     * @return int 最终真实距离值(单位为厘米)
     */
    int calcDistance();

signals:
    /**
     * @brief  Qt信号函数，用于发送原始字符串信号
     * @return QString  获取的原始字符串
     */
    QString sendStringInfo(QString);

    /**
     * @brief  Qt信号函数，发送最终测距信号
     * @param  isRight          当前距离是否已经小于设置的门限距离--需要紧急刹车
     * @param  distance         最终数据真实值
     */
    void sendDistance(bool isRight, int distance);

public:
    int distance;               ///< 最终障碍物距离，取值范围为：20cm ~ 720cm
    QString stringInfo;         ///< 获取到的原始16进制字符串，用于进读取和计算
    QSerialPort *m_serialPort;  ///< 串口类 read distance data
    QString d_byte_str;         ///< 关键距离数据二进制字符串
    QString portName;           ///< 串口Name
    QString portConfigFileName; ///< 串口配置文件名称，主要用于获取设备配置参数
};

#endif // ULTRASONIC_H
