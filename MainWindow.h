#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "SerialPortDialog.h"
#include "PaintWidget.h"
#include "RingBuffer.h"
#include <QMainWindow>
#include <QDataStream>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void print(const QString&);

private:
    Ui::MainWindow *ui;
    void open();
    void openProcessRun();
    void openProcessRunNoGPS();
    void openSerialDialog();
    void openSocketDialog();
    void openInitRouteDialog();
    void openRouteSparseDialog();
    void openTinyCarSerialDialog();
    void openFile();
    QAction* setSerialAction;   //打开串口设置,接收gps
    QAction* setSocketAction;   //打开Socket设置，接收gps
    QAction* initRouteAction;   //初始化路径
    QAction* startRunningAction;//沿着路径行驶,输入数据为GPS
    QAction* startRunningNoGPSAction;//沿着路径行驶,输入数据为高斯坐标系
    QAction* loadGPSDataAction; //加载GPS数据到界面，并且作为初始化路径
    QAction* routeSparseAction; //加载GPS数据到界面，并且作为初始化路径
    QAction* setScaleAction;    //设置比例尺
    QAction* setTinyCarComAction;    //设置比例尺
    GpsRingBuffer* gpsRingBuffer;
    PaintWidget* paintWidget;
signals:
    void sendQPointToPaintWidget(QPointF&);
};

#endif // MAINWINDOW_H
