#ifndef PROCESSRUNDIALOG_H
#define PROCESSRUNDIALOG_H

#define STARTPOINTLONGITUDE 103.958573205
#define STARTPOINTLATITUDE 30.783824800
#define STARTPOINTALTITUDE 500

#include "gpsinfo.h"
#include "RingBuffer.h"
#include "GPSProcesser.h"
#include "GpsBufferConsumeRunThread.h"
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
    void setStartGps(GpsInfo gps);
    void copySetInitRouteList(QList<QPointF>);
    GpsBufferConsumeRunThread* getGpsBufferConsumeRunThread(){return gpsBufferConsumeRunThread;}
    ~ProcessRunDialog();

private:
    Ui::ProcessRunDialog *ui;
    GpsBufferConsumeRunThread* gpsBufferConsumeRunThread;
    GPSProcesser* gpsProcesser;
    QList<QPointF> routePointList;//需要现在主界面加载路径后
    void initGpsProcessRoute(QList<QPointF>);//初始化路径
    void initCoordinateOriginPoint(QPointF);//初始化高斯坐标原点GPS
    void initStartPointByGpsInfo(GpsInfo gps);//初始化车辆起始位置坐标GPS
    void initStartPoint(double longitude,double latitude,double altitude);//初始化车辆起始位置坐标GPS
    void initStartPointToQLable(double longitude,double latitude,double altitude);//同步数据到label界面
signals:
    void sendInitSignal(bool signal);
};

#endif // PROCESSRUNDIALOG_H
