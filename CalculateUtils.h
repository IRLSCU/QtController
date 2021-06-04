/**
 * @file CalculateUtils.h
 * @brief CalculateUtils 类头文件，主要包含各种点计算工具函数
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-04 12:45:42
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
 * 
 * <tr>
 *    <td> 2019-06-04 12:45:42 </td>
 *    <td> 1.0 </td>
 *    <td> zhuhaibo </td>
 *    <td> 完成基础工具点计算类 </td>
 * </tr>
 * <tr>
 *    <td> 2021-06-04 12:45:42 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加文档注释</td>
 * </tr>
 * </table>
 */
#ifndef CALCULATEUTILS_H
#define CALCULATEUTILS_H
#include <QPointF>

/**
 * @brief 定义高精度PI值，精度到小数点后9位
 */
#define PI 3.141592654

/**
 * @brief 计算工具类，主要包含点计算函数
 */
class CalculateUtils
{
public:
    CalculateUtils();
    /**
     * @brief  两点之间欧式距离计算函数
     * @param  p1               二维坐标点p1
     * @param  p2               二维坐标点p2
     * @details 计算公式如下： \n
     *   \f$ res = \sqrt[2]{ (p1.x-p2.x)^2 + (p1.y-p2.y)^2 } \f$
     * @return double           最终欧式距离
     */
    static double distanceTwoPoint(QPointF p1, QPointF p2);
    /**
     * @brief  旋转点逆时针旋转函数
     * @details 旋转点绕旋转中心旋转指定角度
     * @param  center           旋转中心
     * @param  roratePoint      旋转原始数据点
     * @param  angle            旋转角度
     * @return QPointF          旋转后的目标数据点
     */
    static QPointF roratePoint(QPointF center, QPointF roratePoint, double angle);
    /**
     * @brief  计算方向向量
     * @details 以center为起点表示当前车辆行驶方向,根据上一个点和当前定位点，粗略地表示车辆的行驶方向
     * @param  center           起点
     * @param  last             上一个行驶点
     * @param  length           两者之间的行驶长度
     * @return QPointF          最终行驶方向点
     */
    static QPointF calCoursePoint(QPointF center, QPointF last, double length);
    /**
     * @brief  根据order中转向角度，绘制转向方向
     * @details 以center为中心，将coursePoint转动转向角度，仅表示程度值 \n 
     * 从模型上公式上推导，该转向程度值和实际转向值(角速度)都是线性的
     * @param  center           中心
     * @param  coursePoint      目标导航点
     * @param  range            转动角度
     * @return QPointF          最终防线
     */
    static QPointF calRangePoint(QPointF center, QPointF coursePoint, int range);
};

#endif // CALCULATEUTILS_H
