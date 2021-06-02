#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDataStream>

#include "SerialPortDialog.h"
#include "PaintWidget.h"
#include "RingBuffer.h"
#include "CoTrans.h"
/**
 * @brief Qt UI类，根据.ui文件自动生成
 */
namespace Ui
{
    class MainWindow;
}
/**
 * @brief 主要接口实现类,包含主窗口配置类
 * @todo 
 * - 将GPS设置与车辆控制函数进行分离
 * - 统一抽象接口
 * - 进一步封装相关函数
 * 
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief Construct a new Main Window object
     * @param  parent           父指针
     */
    explicit MainWindow(QWidget *parent = 0);
    /**
     * @brief Destroy the Main Window object
     * @details 继承自QMainWindow 主要负责类中的指针销毁
     */
    ~MainWindow();
    /**
     * @brief 打印测试函数
     * @details 过时函数
     */
    void print(const QString &);
    /**
     * @brief Set the Gps Route List object
     * @param  gpsRouteList     gps全局坐标列表
     */
    inline void setGpsRouteList(QList<QPointF> &gpsRouteList) { this->gpsRouteList = gpsRouteList }
    inline QList<QPointF> getGpsRouteList() { return this->gpsRouteList }
    /**
     * @brief Set the Location Route List object
     * @param  locationRouteList 局部相对定位坐标点
     */
    inline void setLocationRouteList(QList<QPointF> &locationRouteList) { this->locationRouteList = locationRouteList }
    inline QList<QPointF> getLocationRouteList() { return this->locationRouteList }

private:
    /**
     * @brief ui成员指针
     */
    Ui::MainWindow *ui;
    /**
     * @brief 开启弹窗，测试接口无实际意义
     * @note 测试接口无实际意义
     */
    void open();
    //GPS
    /**
     * @addtogroup GPS GPS坐标系相关函数 
     * @details 包含GPS全局坐标系的相关函数与成员变量
     * @{
     */
    
    /**
     * @brief 主要按钮函数 
     */
    void openProcessRun();
    /**
     * @brief "串口设置"按钮点击显示函数
     */
    void openSerialDialog();
    /**
     * @brief "socket设置"显示函数
     * @details 主要用于设置监听的socket地址和端口号，用来进行端口监听
     * 获取网络控制指令
     */
    void openSocketDialog();
    /**
     * @brief "初始化路径" 界面控制函数
     */
    void openInitRouteDialog();
    /**
     * @brief "路径稀疏"化控制窗口
     */
    void openRouteSparseDialog();
    /**
     * @brief "小车串口设置"窗口函数
     * @see TinyCarSerialPortDialog
     */
    void openTinyCarSerialDialog();
    /**
     * @brief 打开文件路径函数
     */
    void openFile();
    /**
     * @}
     */

    //XYZ
    /**
     * @addtogroup XYZ SLAM 相对定位坐标系函数
     * @details @details 包含SLAM 相对定位坐标系的相关函数与成员变量
     * @{
     */

    /**
     * @brief 根据相对定位执行GPS显示
     */
    void openProcessRunNoGPS();
    /**
     * @brief 相对定位路径初始化
     */
    void openLocationInitRouteDialog();
    /**
     * @brief 打开坐标记录函数点
     * @details 对应加载路径(XYZ)对应函数
     */
    void openXYZFile();
    /**
     * @brief 打开ROS配置窗口，主要是ROS相关配置参数
     * @note
     *  **使用时请确认ROS master 已经开启**
     * @todo
     *  增加错误信息弹窗
     */
    void openRosConnectionDialog();
    /**
     * @brief 
     */
    void loadCenterGPS();
    void initOrdinate();
    void openRosDialog();
    /**
     * @}
     */

    //gps
    /**
     * @addtogroup GPS GPS操作相关事件响应成员类 
     * @{
     */
    
    /**
     * @brief  打开串口设置,接收gps 
     */
    QAction *setSerialAction;
    /**
     * @brief 打开Socket设置，接收gps
     */
    QAction *setSocketAction;
    /**
     * @brief 初始化路径
     */

    QAction *initRouteAction;
    /**
     * @brief 沿着路径行驶,输入数据为GPS
     */
    QAction *startRunningAction;
    /**
     * @brief 加载GPS数据到界面，并且作为初始化路径
     */
    QAction *loadGPSDataAction;
    /**
     * @brief 稀疏GPS点迹
     */
    QAction *routeSparseAction;
    /**
     * @}
     */
    /**
     * @addtogroup XYZ SLAM 相对定位坐标系函数
     * @{
     */
    
    /**
     * @brief ros设置动作按钮
     */
    QAction *setRosAction;
    /**
     * @brief 初始化路径
     */
    QAction *initXYZRouteAction;
    /**
     * @brief 沿着路径行驶,输入数据为高斯坐标系
     */
    QAction *startRunningXYZAction;
    /**
     * @brief 加载XYZ数据到界面，并且作为初始化路径
     */
    QAction *loadXYZDataAction;
    /**
     * @}
     */
    //common
    /**
     * @brief 设置比例尺
     */
    QAction *setScaleAction;
    /**
     * @brief 设置比例尺
     */
    QAction *setTinyCarComAction;
    /**
     * @brief gps数据坐标环形缓冲区
     */
    GpsRingBuffer *gpsRingBuffer;
    /**
     * @brief 相对坐标环形缓冲区
     */
    LocationRingBuffer *locationRingBuffer;
    /**
     * @brief 显示窗口画布
     */
    PaintWidget *paintWidget;
    /**
     * @brief GPS中心相对点
     */
    GpsInfo centerGPS;
    /**
     * @brief XYZ坐标系中心点
     */
    CCoordinate ordinate;

    /**
     * @brief 可视化GPS坐标列表,真实空间全局坐标
     */
    QList<QPointF> gpsRouteList;
    /**
     * @brief SLAM定位局部坐标，表示相对建图空间坐标位置
     */
    QList<QPointF> locationRouteList;

signals:
    /**
     * @brief 信号函数，发送到达位置坐标点
     */
    void sendQPointToPaintWidget(QPointF &);
};

#endif // MAINWINDOW_H
