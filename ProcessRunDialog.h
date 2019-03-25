#ifndef PROCESSRUNDIALOG_H
#define PROCESSRUNDIALOG_H

#define STARTPOINTLONGITUDE 103.958573205
#define STARTPOINTLATITUDE 30.783824800
#define STARTPOINTALTITUDE 500

#include "RingBuffer.h"
#include "GPSProcesser.h"
#include "PID.h"
#include "GpsBufferConsumeRunThread.h"
#include "ControlOrderSendThread.h"
#include <QDialog>

namespace Ui {
class ProcessRunDialog;
}

class ProcessRunDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ProcessRunDialog(GpsRingBuffer*, QWidget *parent = 0);
    void acceptGpsInfo(GpsInfo& GpsInfo);
    void updateBroswerText(GpsInfo GpsInfo);
    void startProcess();
    void continueProcess();
    void endProcess();
    void saveFile();
    void readFile();
    void setStartGps(GpsInfo gps);
    void copySetInitRouteList(QList<QPointF>);
    void setNextTargetPoint(int i);
    void processGPS(GpsInfo GpsInfo);
    Pid_control* getPid(){return PID;}
    GpsBufferConsumeRunThread* getGpsBufferConsumeRunThread(){return gpsBufferConsumeRunThread;}
    ~ProcessRunDialog();

private:
    Ui::ProcessRunDialog *ui;
    GpsBufferConsumeRunThread* gpsBufferConsumeRunThread;
    ControlOrderSendThread* controlOrderSendThread;
    GPSProcesser* gpsProcesser;
    Pid_control* PID;
    QList<QPointF> routePointList;//需要现在主界面加载路径后
    void initGpsProcessRoute(QList<QPointF>);//初始化路径
    void initCoordinateOriginPoint(QPointF);//初始化高斯坐标原点GPS
    void initStartPointByGpsInfo(GpsInfo gps);//初始化车辆起始位置坐标GPS
    void initStartPoint(double longitude,double latitude,double altitude);//初始化车辆起始位置坐标GPS
    void initStartPointToQLable(double longitude,double latitude,double altitude);//同步数据到label界面


signals:
    void sendInitSignal(bool signal);
    void sendStartPointGPSToPaintWidget(QPointF gpsInfo);
    void sendNextTargetPointToPaintWidget(int target);
    void sendRange(int range);//将车辆转向程度，送往车辆命令发送线程
    void sendSpeed(int speed);//将车辆速度，送往车辆命令发送线程
};

#endif // PROCESSRUNDIALOG_H
