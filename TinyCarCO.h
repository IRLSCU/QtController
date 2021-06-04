/**
 * @file TinyCarCO.h
 * @brief TinyCarCO 基础类头文件
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-04 19:58:42
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
 *    <td> 2021-06-04 19:58:42 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加注释文档 </td>
 * </tr>
 * </table>
 */
#ifndef TINYCARCO_H
#define TINYCARCO_H
#include <QObject>
/**
 * @brief 设置小车的最大速度为 255
 */
#define TINYCARCO_MAX_SPEED 0xFF
/**
 * @brief 设置小车最小速度
 */
#define TINYCARCO_MIN_SPEED 0
/**
 * @brief 小车方向前景
 */
#define TINYCARCO_ORIENTATION_FORWARD 0
/**
 * @brief 小车后退
 */
#define TINYCARCO_ORIENTATION_BACKWARD 1
/**
 * @brief The TinyCarCO class
 * 小车控制指令类
 * @details 详细内容见:
 * - [顶配版四轮两驱差速小车开发手册](https://github.com/IRLSCU/documentation_for_devices/blob/26c5b3dea9700df4b621c363b03d22ba311dca17/%E6%8E%A7%E5%88%B6%E5%B0%8F%E8%BD%A6/%E9%A1%B6%E9%85%8D%E7%89%88%E5%9B%9B%E8%BD%AE%E4%B8%A4%E9%A9%B1%E5%B7%AE%E9%80%9F%E5%B0%8F%E8%BD%A6%E9%99%84%E9%80%81%E8%B5%84%E6%96%99/%E9%A1%B6%E9%85%8D%E7%89%88%E5%9B%9B%E8%BD%AE%E4%B8%A4%E9%A9%B1%E5%B7%AE%E9%80%9F%E5%B0%8F%E8%BD%A6%E5%BC%80%E5%8F%91%E6%89%8B%E5%86%8C.pdf)
 */
class TinyCarCO
{
public:
    /**
     * @brief 角速度与线速度初始化
     */
    void init();
    /**
     * @brief 获取左轮线速度
     * @return quint8 左轮线速度  
     */
    quint8 getLeftSpeed() { return leftSpeed; }
    /**
     * @brief 获取右轮线速度对象
     * @return quint8   右轮线速度
     */
    quint8 getRightSpeed() { return rightSpeed; }
    /**
     * @brief Get the Left Orientation object
     * @return qint8 左轮方向
     */
    qint8 getLeftOrientation() { return leftOrientation; }
    /**
     * @brief Get the Right Orientation object
     * @return qint8 右轮方向
     */
    qint8 getRightOrientation() { return rightOrientation; }
    /**
     * @brief 输出转向信息
     */
    void printInfo();
    /**
     * @brief Set the Left Speed object
     * @param  leftSpeed        左轮速度
     * @return TinyCarCO&       更改后的新值
     */
    TinyCarCO &setLeftSpeed(quint8 leftSpeed);
    /**
     * @brief Set the Right Speed object
     * @param  rightSpeed       右轮速度值
     * @return TinyCarCO& 
     */
    TinyCarCO &setRightSpeed(quint8 rightSpeed);
    /**
     * @brief Set the Left Orientation object
     * @param  leftOrientation  左轮转向
     * @return TinyCarCO&       
     */
    TinyCarCO &setLeftOrientation(qint8 leftOrientation);
    /**
     * @brief Set the Right Orientation object
     * @param  rightOrientation 右轮转向
     * @return TinyCarCO& 
     */
    TinyCarCO &setRightOrientation(qint8 rightOrientation);
    /**
     * @brief 转化成实际的指令数组
     * @return quint8* 速度数组
     */
    quint8 *getCharOrder();

private:
    /**
     * @brief leftSpeed
     * @details left wheel speed,range from 0x00 to 0xFF
     */
    quint8 leftSpeed;
    /**
     * @brief rightSpeed
     * @details right wheel speed,range from 0x00 to 0xFF
     */
    quint8 rightSpeed;
    /**
     * @brief leftOrientation
     * @details left wheel orientation range 0 means forward ,1 means backward
     * 左轮转的方向 0表示向前，1表示向后
     */
    qint8 leftOrientation;
    /**
     * @brief rightOrientation
     * @details right wheel orientation range 0 means forward ,1 means backward
     * 左轮转的方向 0表示向前，1表示向后
     */
    qint8 rightOrientation;
    /**
     * @brief 小车基础指令集合
     */
    quint8 charOrder[10] = {0xFF, 0xFE, 0, 0, 0, 0, 0, 0, 0, 0};
};
#endif // TINYCARCO_H
