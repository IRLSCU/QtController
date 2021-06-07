/**
 * @file CoTrans.h
 * @brief  坐标转换  结构、类 
 * @details GPS数据转换XYZ数据,经纬度转化成笛卡尔坐标系
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-07 12:55:42
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
 *    <td> 2021-06-07 12:55:42 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td>内容</td>
 * </tr>
 * </table>
 */
#ifndef COTRANS_H
#define COTRANS_H

#include <QPointF>
//***************************
// 坐标转换  结构、类
//GPS数据转换XYZ数据,经纬度转化成笛卡尔坐标系
//***************************

//经度-纬度-高度 Coordinate
/**
 * @brief 经纬度数据对象抽象类
 * 
 */
struct LongLat
{
    qreal Lon; ///< Longitude 度
    qreal Lat; ///< Latitude  度
    qreal Hei; ///< Height 海拔高度 米
    /**
     * @brief Construct a new LongLat object
     */
    LongLat()
    {
    }
    /**
     * @brief Construct a new LongLat object
     * @param  lon              经度
     * @param  lat              维度
     * @param  hei              水平高度
     */
    LongLat(qreal lon, qreal lat, qreal hei = 0) : Lon(lon), Lat(lat), Hei(hei)
    {
    }
};

/**
 * @brief CCoordinate_API CCoordinate
 */
class CCoordinate
{
public:
    /**
     * @brief Construct a new CCoordinate object
     */
    CCoordinate();
    /**
     * @brief Destroy the CCoordinate object
     */
    ~CCoordinate() {}

public:
    /**
     * @brief  初始化位置中心 
     * @param  radarheight       高度 单位：米 
     * @param  radarlongitude    longitude 经度坐标，单位：度
     * @param  radarlatitude     latitude  纬度坐标，单位：度
     */
    void InitRadarPara(qreal radarheight, qreal radarlongitude, qreal radarlatitude);
    LongLat GetCenterLL();
    /**
     * @brief  将经纬度坐标点，转换为相对坐标点
     * @param  pos              原始经纬度坐标点对象
     * @return QPointF          相对坐标点对象
     */
    QPointF LongLat2XY(const LongLat &pos);
    /**
     * @brief  将经纬度转换为坐标对象
     * @param  lon              经度坐标
     * @param  lat              纬度坐标
     * @param  hei              高度
     * @return QPointF          转换后的坐标点
     */
    QPointF LongLat2XY(const qreal &lon, const qreal &lat, const qreal &hei);
    /**
     * @brief  将经纬度转换为坐标点
     * @param  lon              经度
     * @param  lat              纬度
     * @return QPointF 
     */
    QPointF LongLat2XY(const qreal &lon, const qreal &lat);
    /**
     * @brief 
     * @param  pos              相对坐标系
     * @return LongLat 
     */
    LongLat XY2LongLat(QPointF pos);
    /**
     * @brief  相对位置坐标转换为经纬度
     * @param  x                
     * @param  y                
     * @return LongLat          经纬度坐标点
     */
    LongLat XY2LongLat(const qreal &x, const qreal &y);
    /**
     * @brief  相对坐标到屏幕坐标系
     * @param  pos              相对坐标
     * @return QPointF          屏幕坐标相对点
     */
    QPointF XY2Screen(QPointF pos);
    /**
     * @brief  屏幕坐标系向相对坐标系进行转换
     * @param  pos              相对坐标点
     * @return QPointF          屏幕做指标点
     */
    QPointF Screen2XY(QPointF pos);
    /**
     * @brief  屏幕坐标点输出为x,y值
     * @param  x                相对坐标值 x
     * @param  y                相对坐标值 y
     * @return QPointF          输出坐标值
     */
    QPointF Screen2XY(const qlonglong &x, const qlonglong &y);
    /**
     * @brief  
     * @param  pos              经纬度转换为屏幕坐标点
     * @return QPointF          最终屏幕绘图坐标点
     */
    QPointF LongLat2Screen(LongLat pos);
    /**
     * @brief  屏幕坐标点到经纬度坐标系转换
     * @param  pos              屏幕上坐标点
     * @return LongLat          最终经纬度上坐标点
     */
    LongLat Screen2LongLat(QPointF pos);
    /**
     * @brief  屏幕绘图坐标系向经纬度坐标系进行转换
     * @param  x                屏幕上坐标点x
     * @param  y                屏幕上坐标点y
     * @return LongLat          最终经纬度坐标点
     */
    LongLat Screen2LongLat(const qlonglong &x, const qlonglong &y);

public:
    qreal m_fixedScale;                             ///< 固定放大倍数
    qreal m_scale;                                  ///< 屏幕显示放大倍数
    QPointF m_DspCenter;                            ///< 显示中心
private:
    qreal m_Hei;                                    ///< Height	海拔高度
    qreal m_Lon;                                    ///< Longitude 经度 度
    qreal m_Lat;                                    ///< Latitude	纬度 度
    qreal m_longsin, m_longcos, m_latsin, m_latcos; ///< 经纬度的三角函数
    qreal m_Radius;                                 ///< 地球等效半径
    qreal m_RadarHei;                               ///< 目标距球心距离
    LongLat m_LLPos;                                ///< 绝对坐标，经纬度，1/10^4 度
    QPointF m_XYPos;                                ///< 相对于中心点的平面坐标，米
    QPointF m_screen;                               ///< UI桌面控制对象
};

#endif
