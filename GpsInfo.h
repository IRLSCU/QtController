/**
 * @file GpsInfo.h
 * @brief  GPS基础信息抽象类GpsInfo
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-04 19:43:39
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
 *    <td> 2021-06-04 19:43:39 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加文档注释 </td>
 * </tr>
 * </table>
 */
#ifndef GPSINFO_H
#define GPSINFO_H

#include <QDebug>

/**
 * @brief 海里/公里转换参数
 * @details 一海里=1.852公里 
 */
#define KNOTS2KPH 1.852

/**
 * @brief GPS相关信息
 * @details GPS信息与操作相关类，包含主要数据与相关操作
 */
struct GpsInfo
{
    /**
     * @brief Construct a new Gps Info object
     * @param  lon              经度
     * @param  lat              纬度
     * @param  quality          质量
     * @param  time             时间
     * @param  date             日期
     * @param  alt              高度
     * @param  speed            速度
     * @param  course           航向
     */
    GpsInfo(double lon = 0, double lat = 0, int quality = 0, double time = 0, unsigned long date = 0, double alt = 0, double speed = 0, double course = 0)
        : longitude(lon), latitude(lat), quality(quality), time(time), date(date), altitude(alt), speed(speed * KNOTS2KPH), course(course) {}
    /**
     * @brief 打印信息，输出对应的字符串
     */
    void printInfo()
    {
        qDebug() << toString();
    }
    /**
     * @brief  toString 序列化函数
     * @return QString 序列化后的字符串
     */
    QString toString()
    {
        QString content = QObject::tr("%1 %2 %3 %4 %5 %6 %7 %8").arg(longitude, 8, 'f', 10).arg(latitude, 8, 'f', 10).arg(quality).arg(time).arg(date).arg(altitude).arg(speed).arg(course);
        return content;
    }
    /**
     * @brief  GPS info反序列化函数
     * @param  str              原始字符串
     * @return GpsInfo          GpsInfo 基础信息
     */
    static GpsInfo StringToGpsInfo(QString str)
    {
        QStringList strs = str.split(" ");
        GpsInfo info;
        info.longitude = strs.at(0).toDouble();
        info.latitude = strs.at(1).toDouble();
        info.quality = strs.at(2).toInt();
        info.time = strs.at(3).toDouble();
        info.date = strs.at(4).toULong();
        info.altitude = strs.at(5).toDouble();
        info.speed = strs.at(6).toDouble();
        info.course = strs.at(7).toDouble();

        return info;
    }
    double longitude;   ///< 经度
    double latitude;    ///< 纬度
    int quality;        ///< 数据质量准确度
    double time;        ///< 时分秒
    unsigned long date; ///< 日月年
    double altitude;    ///< 高度
    double speed;       ///< 速度
    double course;      ///< 航向
};
/**
 * @brief 提供默认拷贝构造函数
 */
Q_DECLARE_METATYPE(GpsInfo)
#endif // GPSINFO_H
