/**
 * @file GPSProcesser.h
 * @brief GPSProcesser 类头文件
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-04 19:56:41
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
 *    <td> 2021-06-04 19:56:41 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td>内容</td>
 * </tr>
 * </table>
 */
#ifndef GPS_PROCESSER
#define GPS_PROCESSER

#include "Processer.h"
#include "CoTrans.h"
#include "GpsInfo.h"
/**
 * @brief GPS信息处理相关类
 */
class GPSProcesser
{

public:
    GPSProcesser();
    ~GPSProcesser();
    static void initCCoordinate();
    bool initRoute(QList<QPointF>);                        //初始化gps_route QPointF(longitude,latitude)
    int initStartPoint(double longitude, double latitude); //通过当前位置设定起始点，返回在固定路径中最近的位置,若无则返回0;
    int setNextTargetPoint(int i);
    double startProcess(GpsInfo current, int *target, int *status); //通过当前点返回其离直线的距离，用来纠偏
    GaussGPSData gpsData2Gauss(double gpslongitude, double gpslatitude, int quality);
    GaussGPSData gpsData2Gauss(GpsInfo gpsInfo);
    static CCoordinate cCoordinate;

    Processer processer;
};
#endif // GPS_PROCESSER
