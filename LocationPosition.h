/**
 * @file LocationPosition.h
 * @brief LocationPosition 类主要头文件
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-04 21:00:02
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
 *    <td> 2021-06-04 21:00:02 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加文档注释 </td>
 * </tr>
 * </table>
 */
#ifndef LOCATIONPOSITION_H
#define LOCATIONPOSITION_H

#include <QDebug>
/**
 * @brief 定位坐标相关抽象类
 */
struct LocationPosition
{
    /**
     * @brief 定位信息抽象类
     * @param  x                x坐标值
     * @param  y                y坐标值
     * @param  z                z坐标值
     * @param  speed            速度
     * @param  course           方向
     * @param  precision        准确度
     * @param  timestamp        时间戳
     */
    LocationPosition(double x = 0, double y = 0, double z = 0, double speed = 0, double course = 0, double precision = 1, unsigned long timestamp = 0)
        : x(x), y(y), z(z), speed(speed), course(course), precision(precision), timestamp(timestamp) {}
    void printInfo()
    {
        qDebug() << toString();
    }
    /**
     * @brief 序列化函数
     * @return QString 
     */
    QString toString()
    {
        QString content = QObject::tr("%1 %2 %3 %4 %5 %6 %7").arg(x, 2, 'f', 4).arg(y, 2, 'f', 4).arg(z, 2, 'f', 4).arg(speed, 2, 'f', 2).arg(course, 2, 'f', 2).arg(precision, 2, 'f', 2).arg(timestamp);
        return content;
    }
    /**
     * @brief 反序列化
     * @param  str              序列化后的字符串
     * @return LocationPosition 对象
     */
    static LocationPosition stringToLocation(QString str)
    {
        LocationPosition location;
        QStringList strs = str.split(" ");

        location.x = strs.at(0).toDouble();
        location.y = strs.at(1).toDouble();
        location.z = strs.at(2).toDouble();
        location.speed = strs.at(3).toDouble();
        location.course = strs.at(4).toDouble();
        location.precision = strs.at(5).toDouble();
        location.timestamp = strs.at(6).toULong();
        return location;
    }
    double x;                ///< x坐标值
    double y;                ///< y坐标值
    double z;                ///< z坐标值
    double speed;            ///< 速度
    double course;           ///< 航向
    double precision;        ///< 精度
    unsigned long timestamp; ///< 时间戳
};
#endif // LOCATIONPOSITION_H
