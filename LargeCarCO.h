/**
 * @file LargeCarCO.h
 * @brief LargeCarCO 类头文件
 * @details 包含校园载人公交车控制相关指令和参数
 * @note 校园载人大车已不再使用，此仅仅有参考价值
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-04 20:18:28
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
 *    <td> 2021-06-04 20:18:28 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加注释文档 </td>
 * </tr>
 * </table>
 */
#ifndef LARGECARCO_H
#define LARGECARCO_H

#include <QDebug>
/**
 * @brief 设置大车控制最大速度 标志位
 */
#define LARGECARCO_MAX_SPEED 0
/**
 * @brief 最小速度串口通信标志位
 */
#define LARGECARCO_MIN_SPEED 127
/**
 * @brief 最大制动数值
 */
#define LARGECARCO_MAX_BRAKE 255
/**
 * @brief 最小制动数值
 */
#define LARGECARCO_MIN_BRAKE 128
/**
 * @brief 最大转弯范围
 * @details 方向盘正向最大转动角度
 */
#define LARGECARCO_MAX_TURN_RANGE 32767
/**
 * @brief 最小转弯范围
 * @details 方向盘反向最大转动角度
 */
#define LARGECARCO_MIN_TURN_RANGE -32768
/**
 * @brief 定义档位--0--空档
 */
#define LARGECARCO_GEAR_ZERO 0
/**
 * @brief 定义档位--1--前进档
 */
#define LARGECARCO_GEAR_FORWARD 1
/**
 * @brief 定义档位--1--后退档
 */
#define LARGECARCO_GEAR_BACKWARD 2
/**
 * @brief 灯光控制标志位： 0不亮
 * @details 灯光控制标志位 0不亮 1右转向灯亮 2左转向灯亮 3应急灯亮
 */
#define LARGECARCO_LIGHT_ALL_OFF 0
/**
 * @brief 灯光控制标志位： 1右转向灯亮
 * @details 灯光控制标志位 0不亮 1右转向灯亮 2左转向灯亮 3应急灯亮
 */
#define LARGECARCO_LIGHT_RIGHT_ON 1
/**
 * @brief 灯光控制标志位： 2左转向灯亮
 * @details 灯光控制标志位 0不亮 1右转向灯亮 2左转向灯亮 3应急灯亮
 */
#define LARGECARCO_LIGHT_LEFT_ON 2
/**
 * @brief 鸣笛：0-关闭鸣笛
 * @details 鸣笛控制， 0-关闭鸣笛，1-开启鸣笛
 */
#define LARGECARCO_HORN_OFF 0
/**
 * @brief 鸣笛：1-开启鸣笛
 * @details 鸣笛控制， 0-关闭鸣笛，1-开启鸣笛
 */
#define LARGECARCO_HORN_ON 1
/**
 * @brief 定义控制长度，单位为字节
 */
#define LARGECARCO_LENGTH 8
/**
 * @brief 校园载人公交车控制指令抽象类
 * @details 大车控制类抽象，主要是通过can口进行控制 \n
 * 指令总长度为  @link LARGECARCO_LENGTH
 */
class LargeCarCO
{
public:
    /**
     * @brief 大车控制指令类构造函数
     */
    LargeCarCO();
    /**
     * @brief 析构函数
     */
    ~LargeCarCO();

    /**
     * @brief 设置车辆速度
     * @details 车辆速度--通过油门范围控制--相当于加速度 \n
     * 加速范围（127，0）仅代表程度，不表示具体加速度。127表示最小加速度，0最大加速
     * @param  speed            速度值
     * @return LargeCarCO&      更改后的速度指令对象
     */
    LargeCarCO &setSpeed(quint8 speed);

    /**
     * @brief 设置刹车减速值
     * @details 减速范围（128，255）仅代表程度，不表示具体加速度。128最小减速度，255最大减速度
     * @param  brake            减速值
     * @return LargeCarCO& 
     */
    LargeCarCO &setBrake(quint8 brake);

    /**
     * @brief 设置转向值
     * @details 转向范围（-32768，32767）负代表左转，正代表右转。数值仅表示程度
     * @param  turnRange        转向值
     * @return LargeCarCO& 
     */
    LargeCarCO &setTurnRange(qint16 turnRange);

    /**
     * @brief 设置档位值
     * @details 三种挡位0空挡，1表示前进挡，2表示后退挡
     * @param  gear             档位值
     * @return LargeCarCO& 
     */
    LargeCarCO &setGear(quint8 gear);

    /**
     * @brief 设置灯光控制值
     * @details 灯光 0不亮 1右转向灯亮 2左转向灯亮 3应急灯亮
     * @param  signal           灯光控制值
     * @return LargeCarCO& 
     */
    LargeCarCO &setSignal(quint8 signal);
    /**
     * @brief 设置鸣笛值
     * @details 鸣笛 0静音 1鸣笛
     * @param  horn             鸣笛值
     * @return LargeCarCO& 
     */
    LargeCarCO &setHorn(quint8 horn);
    /**
     * @brief 紧急制动，控制指令均归0 
     */
    void emergencyBraking();
    /**
     * @brief 所有的指令初始化到0
     */
    void init();
    /**
     * @brief 获取速度值
     * @return quint8 速度值
     */
    inline quint8 getSpeed() { return speed; }
    /**
     * @brief 获取加速度值
     * @return qint16 加速度值
     */
    inline qint16 getTurnRange() { return turnRange; }
    /**
     * @brief 获取档位
     * @return quint8 档位值
     */
    inline quint8 getGear() { return gear; }
    /**
     * @brief 获取灯光值
     * @return quint8 灯光值
     */
    inline quint8 getSignal() { return signal; }
    /**
     * @brief 设置鸣笛
     * @return quint8 鸣笛值
     */
    inline quint8 getHorn() { return horn; }
    /**
     * @brief 转化成实际的指令
     * @return quint8* 串口指令最终字符串数组指针
     */
    quint8 *getCharOrder();
    /**
     * @brief 打印信息
     */
    void printInfo();

private:
    quint8 speed = 127;                                ///< 速度，占第0，1字节
    qint16 turnRange = 0;                              ///< 转向，占第2字节
    quint8 gear = 0;                                   ///< 挡位，占第3字节
    quint8 signal = 0;                                 ///< 灯光,占第四字节
    quint8 horn = 0;                                   ///< 鸣笛，占第五字节
    quint8 charOrder[8] = {0, 0, 0x80, 0, 0, 0, 0, 0}; ///< 原始指令数据
};

#endif // LARGECARCO_H
