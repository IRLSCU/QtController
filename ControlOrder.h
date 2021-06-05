/**
 * @file ControlOrder.h
 * @brief ControlOrder 类统一抽象指令
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-04 21:39:04
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
 *    <td> 2021-06-04 21:39:04 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加文档注释 </td>
 * </tr>
 * </table>
 */
#ifndef CONTROLORDER_H
#define CONTROLORDER_H

#include <QObject>

#include "LargeCarCO.h"
#include "TinyCarCO.h"
#include "GpsInfo.h"
#include "LocationPosition.h"
/**
 * @brief 控制指令，最大速度
 */
#define CONTROLORDER_MAX_SPEED 32767
/**
 * @brief 控制指令，速度为零
 */
#define CONTROLORDER_ZERO_SPEED 0
/**
 * @brief 控制指令，最小速度--反向最大速度
 */
#define CONTROLORDER_MIN_SPEED -32768
/**
 * @brief 最大转向值，正向最大值
 */
#define CONTROLORDER_MAX_TURN_RANGE 32767
/**
 * @brief 最小转向值，反向最小值
 */
#define CONTROLORDER_MIN_TURN_RANGE -32768
/**
 * @brief 档位控制 0 空档
 */
#define CONTROLORDER_GEAR_ZERO 0
/**
 * @brief 档位控制 1 前进档
 */
#define CONTROLORDER_GEAR_FORWARD 1
/**
 * @brief 档位控制 2 后退档
 */
#define CONTROLORDER_GEAR_BACKWARD 2
/**
 * @brief 关闭所有灯光
 */
#define CONTROLORDER_LIGHT_ALL_OFF 0
/**
 * @brief 打开右等
 */
#define CONTROLORDER_LIGHT_RIGHT_ON 1
/**
 * @brief 打开左灯
 */
#define CONTROLORDER_LIGHT_LEFT_ON 2
/**
 * @brief 打开鸣笛
 */
#define CONTROLORDER_HORN_OFF 0
/**
 * @brief 关闭鸣笛
 */
#define CONTROLORDER_HORN_ON 1

/**
 * @brief The ControlOrder class
 * control order to control auto car
 * can transfer to bigCarOrder or smallCalOrder's order
 * @todo 设置统一接口，对代码进行精简，通过对接口的统一实现结合策略模式进行代码精简
 */
class ControlOrder
{
public:
    /**
     * @brief Construct a new Control Order object
     */
    ControlOrder() {}
    /**
     * @brief Destroy the Control Order object
     */
    ~ControlOrder() {}

    /**
     * @brief 设置车辆速度
     * @details 车辆速度--通过油门范围控制--相当于加速度 \n
     * 加速范围（-32768,32767）仅代表程度，不表示具体加速度。-32768表示最大刹车的加速度，32767最大加速
     * @param  speed            速度值
     * @return LargeCarCO&      更改后的速度指令对象
     */
    ControlOrder &setSpeed(int speed);
    /**
     * @brief 设置转向值
     * @details 转向范围（-32768，32767）负代表左转，正代表右转。数值仅表示程度
     * @param  turnRange        转向值
     * @return LargeCarCO& 
     */
    ControlOrder &setTurnRange(int turnRange);
    /**
     * @brief 设置上一个转向范围
     * @param  lastTurnRange    上一个转向范围值
     * @return ControlOrder&    更改后的转向命令
     */
    ControlOrder &setLastTurnRange(int lastTurnRange);

    /**
     * @brief 设置档位值
     * @details 三种挡位0空挡，1表示前进挡，2表示后退挡
     * @param  gear             档位值
     * @return LargeCarCO& 
     */
    ControlOrder &setGear(qint8 gear);

    /**
     * @brief 设置灯光控制值
     * @details 灯光 0不亮 1右转向灯亮 2左转向灯亮 3应急灯亮
     * @param  signal           灯光控制值
     * @return LargeCarCO& 
     */
    ControlOrder &setLightSignal(qint8 signal);
    /**
     * @brief 设置鸣笛值
     * @details 鸣笛 0静音 1鸣笛
     * @param  horn             鸣笛值
     * @return LargeCarCO& 
     */
    ControlOrder &setHorn(quint8 horn);
    /**
     * @brief 设置GPS定位信息
     * @param  gps              GPS定位细腻
     */
    void setGpsInfo(GpsInfo gps);
    /**
     * @brief 设置相对定位信息
     * @param  location         定位信息
     */
    void setLocation(LocationPosition location);
    /**
     * @brief 初始化所有数据
     */
    void init();

    /**
     * @brief Get the Speed object
     * @return qint16 speed
     */
    inline qint16 getSpeed() { return speed; }
    /**
     * @brief Get the Last Turn Range object
     * @return qint16 
     */
    inline qint16 getLastTurnRange() { return lastTurnRange; }
    /**
     * @brief Get the Turn Range object
     * @return qint16 
     */
    inline qint16 getTurnRange() { return turnRange; }
    /**
     * @brief Get the Gear object
     * @return qint8 
     */
    inline qint8 getGear() { return gear; }
    /**
     * @brief Get the Light Signal object
     * @return qint8 
     */
    inline qint8 getLightSignal() { return lightSignal; }
    /**
     * @brief Get the Horn object
     * @return qint8 
     */
    inline qint8 getHorn() { return horn; }
    /**
     * @brief Get the Gps Info object
     * @return GpsInfo 
     */
    inline GpsInfo getGpsInfo() { return gpsInfo; }
    /**
     * @brief Get the Location object
     * @return LocationPosition 
     */
    inline LocationPosition getLocation() { return location; }
    /**
     * @brief 打印指令信息
     */
    void printInfo();

    /**
     * @brief  将标准控制指令转换为小车控制指令
     * @details 主要数据与属性控制如下：
     * Normal       ---->       Tiny 主要转换关键代码如下： \n
     * ```cpp
     * speed[-32768,-1]                     leftSpeed&&rightSpeed=0//刹车
     * speed[0，32767]                      leftSpeed&&rightSpeed[0,255]
     * turnRange[0,32767]                   rightSpeed=(1-k1)*leftSpeed,k1[0,1]//右转
     * turnRange[-1,-32768]                 leftSpeed=(1-k2)*rightSpeed,k2[0,1]//左转
     * if(lastTurnRange*curTurnRange<0)     leftSpeed=rightSpeed=max(leftSpeed,rightSpeed)
     * gear==2                              leftOrientation=rightOrientation=1
     * gear!=2                              leftOrientation=rightOrientation=1
     * ```
     * @param  normalCO         标准控制指令
     * @param  tinyCarCO        小车控制指令详细见 @see TinyCarCO
     */
    static void NormalCO2TinyCarCO(ControlOrder &normalCO, TinyCarCO &tinyCarCO);
    /**
     * @brief  将标准控制转换为大车控制指令
     * @details   转换的规则: \n
     * ```cpp
     * Normal     ---->      Large
     * speed[-32768~-1]             [255~128]
     * speed[0~32767]               [127~0]
     * turnRange(-32768,32767)      (-32768,32767)
     * gear(0,1,2)                  (0,1,2)
     * lightSignal(0,1,2)           (0,1,2)
     * horn(0,1)                    (0,1)
     * ```
     * 
     * @param  normalCO         标准控制指令
     * @param  largeCarCO       大车控制指令详细见 @see LargeCarCO
     */
    static void NormalCO2LargeCarCO(ControlOrder &normalCO, LargeCarCO &largeCarCO);

    /**
     * @brief  将小车控制指令转换为标准指令
     * @details 主要数据与属性控制如下：
     * Normal        <----      Tiny
     * ```cpp
     * speed[-32768]            leftSpeed&&rightSpeed=0//刹车
     * speed[0，32767]          max(leftSpeed,rightSpeed) [0,255]
     * turnRange[0,32767]       leftSpeed>rightSpeed,k1=rightSpeed/leftSpeed,k1[0,1]//右转
     * turnRange[-1,-32768]     rightSpeed>leftSpeed,k2=leftSpeed/rightSpeed,k2[0,1]//左转
     * gear==2                  leftOrientation=rightOrientation=1
     * gear!=2                  leftOrientation=rightOrientation=1
     * ```
     * @param  tinyCarCO        小车控制指令
     * @param  normalCO         标准控制指令
     * 
     */
    static void TinyCarCO2NormalCO(TinyCarCO &tinyCarCO, ControlOrder &normalCO);
     /**
     * @brief  转大车控制指令换为标准指令
     * @details   转换的规则: \n
     * ```cpp
     * Normal     ---->      Large
     * speed[-32768~-1]             [255~128]
     * speed[0~32767]               [127~0]
     * turnRange(-32768,32767)      (-32768,32767)
     * gear(0,1,2)                  (0,1,2)
     * lightSignal(0,1,2)           (0,1,2)
     * horn(0,1)                    (0,1)
     * ```
     * 
     * @param  largeCarCO       大车控制指令详细见 @see LargeCarCO
     * @param  normalCO         标准控制指令
     */
    static void LargeCarCO2NormalCO(LargeCarCO &largeCarCO, ControlOrder &normalCO);

private:
    /**
     * @brief speed
     * @range: from -32768 to 32767
     */
    qint16 speed = 0;
    /**
     * @brief turnRange
     * @range: from -32768 to 32767
     */
    qint16 turnRange = 0;
    /**
     * @brief 最后转向范围
     */
    qint16 lastTurnRange = 0;
    /**
     * @brief gear
     * @range: 0空挡，1前进挡，2后退档
     */
    qint8 gear = 0;
    /**
     * @brief lightSignal
     * @range: 0关闭所有灯光，1右转向灯亮，2左转向灯亮
     */
    qint8 lightSignal = 0;
    /**
     * @brief horn 鸣笛
     * @range: 0静音 1鸣笛
     */
    qint8 horn = 0;
    /**
     * @brief gps基础信息
     * @details 用于记录当前位置信息，方便查询
     */
    GpsInfo gpsInfo;
    /**
     * @brief 定位基本信息
     * @details 用于记录当前位置信息，方便查询
     */
    LocationPosition location;
};
#endif // CONTROLORDERINTERFACE_H
