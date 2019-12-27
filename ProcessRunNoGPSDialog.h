#ifndef PROCESSRUNNOGPSDIALOG_H
#define PROCESSRUNNOGPSDIALOG_H

#define STARTPOINTLONGITUDE 103.958573205
#define STARTPOINTLATITUDE 30.783824800
#define STARTPOINTALTITUDE 500

//#define SENDTOROS

#include "RingBuffer.h"
#include "Processer.h"
#include "PID.h"
#include "LocationBufferConsumeRunThread.h"
#ifdef SENDTOROS
    #include "ControlOrderSendToRosThread.h"
#else
    #include "ControlOrderSendThread.h"
#endif
#include <QDialog>

namespace Ui {
class ProcessRunDialog;
}

class ProcessRunNoGPSDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ProcessRunNoGPSDialog(LocationRingBuffer*, QWidget *parent = 0);
    void initStartPointByLocation(LocationPosition location);
    void updateBroswerText(LocationPosition location);
    void startProcess();
    void continueProcess();
    void endProcess();
    void saveFile();
    void readFile();
    void copySetInitRouteList(QList<QPointF>);
    void setNextTargetPoint(int i);
    void processLocation(LocationPosition location);
    void initStartPointToQLable(double,double,double);
    void initStartPoint(double x, double y, double z);
    Pid_control* getPid(){return PID;}
    LocationBufferConsumeRunThread* getLocationBufferConsumeRunThread(){return locationBufferConsumeRunThread;}
    ~ProcessRunNoGPSDialog();

private:
    Ui::ProcessRunDialog *ui;
    LocationBufferConsumeRunThread* locationBufferConsumeRunThread;
    /**
     * @brief calRange 将当前横向偏差和当前与未来的姿态偏差(车辆方向)做一个加权平均
     * @param rangeCrossError 当前横向偏差
     * @param rangeAngleError 未来的姿态偏差做一个加权平均
     */
    double calRange(double rangeCrossError,double rangeAngleError);
#ifdef SENDTOROS
    ControlOrderSendToRosThread* controlOrderSendThread;
#else
    ControlOrderSendThread* controlOrderSendThread;
#endif
    Processer* processer;
    Pid_control* PID;
    QList<QPointF> routePointList;//需要现在主界面加载路径后
signals:
    void sendInitSignal(bool signal);
    void sendStartPointLocationToPaintWidget(QPointF location);
    void sendNextTargetPointToPaintWidget(int target);
    void sendRange(int range);//将车辆转向程度，送往车辆命令发送线程
    void sendSpeed(int speed);//将车辆速度，送往车辆命令发送线程
    void sendLocationInfo(LocationPosition location);
    void sendDoNothingSignal(bool signal);
};

#endif // PROCESSRUNNOGPSDIALOG_H
