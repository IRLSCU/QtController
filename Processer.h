/**
 * @file Processer.h
 * @brief GPS坐标数据点处理相关 Processer  
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-05 11:09:17
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
 *    <td> 2021-06-05 11:09:17 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加文档注释 </td>
 * </tr>
 * </table>
 */
#ifndef PROCESSER
#define PROCESSER

#include <assert.h>
#include <vector>
#include <stack>
#include <string.h>
#include <fstream>
#include <iostream>
#include <QList>
#include <QPointF>

/**
 * @brief 定义类中常量
 */
#define IN_DISTANCE 1.5
/**
 * @brief 
 */
#define FIXED_SOLUTION 4
#define TARGETDISTANCE 1

/**
 * @brief 高斯坐标系下数据
 */
struct GaussGPSData
{
    /**
     * @brief Construct a new GaussGPS Data object
     */
    GaussGPSData() {}
    /**
     * @brief Construct a new GaussGPS Data object
     * @param  x                纬度坐标
     * @param  y                经度坐标
     * @param  quality          质量参数
     */
    GaussGPSData(double x, double y, double quality = FIXED_SOLUTION) : m_x(x), m_y(y), m_quality(quality) {}
    /**
     * @brief Set the Gps Data object
     * @param  x                纬度坐标
     * @param  y                经度坐标
     * @param  quality          质量参数
     */
    void setGpsData(double x, double y, double quality)
    {
        m_x = x;
        m_y = y;
        m_quality = quality;
    }

    double m_x;       ///< 经度
    double m_y;       ///< 维度
    double m_quality; ///< 质量
};
/**
 * @brief 定义线段操作
 */
struct Line
{
    /**
     * @brief Construct a new Line object
     * @param  start            线段起始坐标点
     * @param  end              线段结束坐标点
     */
    Line(GaussGPSData start, GaussGPSData end)
    {
        m_start = start;
        m_end = end;
    }

    GaussGPSData m_start; ///< 线段起始坐标点
    GaussGPSData m_end;   ///< 线段起始坐标点
};
/**
 * @brief 别名
 */
typedef GaussGPSData VectorG;
/**
 * @brief GPS坐标点相关处理操作类
 */
class Processer
{

public:
    /**
     * @brief Construct a new Processer object
     */
    Processer();
    /**
     * @brief Destroy the Processer object
     */
    ~Processer();
    /**
     * @brief  初始化路径函数
     * @param  route            高斯类型数据系列点
     * @return true             初始化成功
     * @return false            初始化失败
     */
    bool initRoute(QList<QPointF> route);
    /**
     * @brief  通过当前位置设定起始点，返回在固定路径中最近的位置,若无则返回0;
     * @param  current          当前位置坐标点
     * @return int              起始点相对数组下标索引，没有就返回0--表示从起点开始
     */
    int initStartPoint(GaussGPSData current);
    /**
     * @brief Set the Next Target Point object
     * @details 用于更新
     * @param  i                目标点坐标索引
     * @return int              
     */
    int setNextTargetPoint(int i);
    /**
     * @brief 通过当前点返回其离直线的距离，用来纠偏
     * @param current           传入当前高斯坐标坐标
     * @param target            传出目标位置在route的位置
     * @param status            传出状态(1：到达终点；0：未达终点)
     * @return double 
     */
    double startProcess(GaussGPSData current, int *target, int *status);
    /**
     * @brief  通过当前点返回其离直线的距离，用来纠偏,目前未采用
     * @param  current          传入当前高斯坐标坐标
     * @return double 
     */
    double startProcess2(GaussGPSData current);
    /**
     * @brief calTargetYawDiff 获取目标的姿态角和当前姿态角的差
     * @param current           传入当前高斯坐标坐标
     * @param dis               设定的当前和目标位置的距离(方便调试)
     * @return double           角度值
     * @todo 可通过slam获取目标当前的位姿
     */
    double calTargetYawDiff(GaussGPSData current);

private:
    /**
     * @brief 获取两点之间的距离
     * @param  start            开始坐标点
     * @param  end              结束坐标点
     * @return double           最终距离输出值
     */
    static double getPointDistance(GaussGPSData start, GaussGPSData end);
    /**
     * @brief 获取线段(直线)长度
     * @param  line             线段对象
     * @return double           相对线段长度
     */
    static double getLineDistance(Line line);
    /**
     * @brief  计算两个线段的点积(正交基)
     * @param  line1            线段1
     * @param  line2            线段2
     * @return double           最终计算结果
     */
    static double pointProduct(Line line1, Line line2);
    /**
     * @brief  判断目标点距离是否在起点指定范围内
     * @details 常用来判断是否已经到达目标点
     * @param  source           起始坐标点
     * @param  destination      目标坐标点
     * @param  range            最大范围
     * @return true             在指定范围内
     * @return false            不在指定范围内
     */
    static bool isInArea(const GaussGPSData source, const GaussGPSData destination, const double range);
    /**
     * @brief  计算点到线段的最短距离
     * @details 相关计算公式如下: \n
     * \f$ Ax+ By + C = 0 \f$ \n 
     * \f$ res = |Ax_0+By_0+C| / \sqrt(A^2+B^2) \f$
     * 
     * @param  point            坐标点
     * @param  line             线段
     * @return double           距离结果
     */
    static double calPointFromLineDistance(const GaussGPSData point, const Line line);
    /**
     * @brief  通过向量计算距离
     * @details 
     * 向量AB=line，点C在向量AB的左边(-1)，右边（1），在向量上0 \n 
     * 转化成向量AB与向量AC的向量积 
     * @param  point            坐标点
     * @param  line             线段
     * @return double           距离结果
     */
    static double calPointFromLineDirector(const GaussGPSData point, const Line line);
    /**
     * @brief  检查两点是否都在线段同一测
     * @param  point1           数据点1
     * @param  point2           数据点2
     * @param  line             线段值
     * @return true             是在同一侧
     * @return false            不是在同一侧
     */
    static bool pointsInSameSide(GaussGPSData point1, GaussGPSData point2, Line line);
    /**
     * @brief  获取过一条`point`垂直于`line`的直线
     * @param  line             目标直线
     * @param  point            目标点
     * @return Line             结果直线
     */
    static Line getCorssLine(Line line, GaussGPSData point);
    /**
     * @brief  计算line1 到line2的转向角 
     * @param  line1            线段一
     * @param  line2            线段二
     * @return double           角度数值
     */
    static double calLinesAngle(Line line1, Line line2);
    /**
     * @brief  计算线段斜率
     * @param  line             线段
     * @return double           最终斜率值
     */
    static double calLineK(Line line);
    /**
     * @brief  k1到k2的转向角
     * @param  k1               斜率1
     * @param  k2               斜率2
     * @return double           最终数值
     */
    static double calLinesAngle(double k1, double k2);
    /**
     * @brief 计算两个向量之间的夹角
     * @param  v1               向量1
     * @param  v2               向量2
     * @return double           夹角值，注意这里是角度制
     */
    static double calAngle(VectorG v1, VectorG v2);
    /**
     * @brief getCurruntYawVector
     * @param current
     * @param vector
     * @return int -1 表示当前方位角异常(计算方式取前一段距离的点可能取不到) 1表示正常
     */
    int getCurrentYawVector(GaussGPSData current, VectorG &vector, int &curIdx);
    /**
     * @brief 获取当前点后第nextIdx个点
     * @param  current          当前向量点
     * @param  vector           获取到的目标点
     * @param  nextIdx          下一目标点索引，由函数进行更新
     * @return int              函数执行是否成功 -1 表示失败 1表示成功
     */
    int getNextYawVector(GaussGPSData current, VectorG &vector, int &nextIdx);

    std::vector<GaussGPSData> gps_route;     ///< 固定路径的路径
    std::vector<GaussGPSData> passwayBuffer; ///< 已到达到过的坐标点路径集合
    GaussGPSData current_gps;                ///< 当前点
    GaussGPSData last_gps;                   ///< 上一个GPS点
    GaussGPSData current_route_gps;          ///< 距离路径上最近的GPS点
    GaussGPSData next_route_gps;             ///< 全局规划目标点
    GaussGPSData next_gps;                   ///< 下一个目标位置
    int current_point_count;                 ///< vector上的位置
    int sum_gps_point;                       ///< gps取点总和
};
#endif // PROCESSER
