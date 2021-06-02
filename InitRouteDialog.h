#ifndef INITROUTEDIALOG_H
#define INITROUTEDIALOG_H

#include <QDialog>
#include "GpsInfo.h"
#include "GpsBufferReadInitRouteThread.h"
#include "RingBuffer.h"

//初始化路径界面
namespace Ui
{
    class InitRouteDialog;
}

class InitRouteDialog : public QDialog
{
    Q_OBJECT

public:
    explicit InitRouteDialog(GpsRingBuffer *, QWidget *parent = 0);
    void acceptGpsInfo(GpsInfo &GpsInfo);
    void updateBroswerText(GpsInfo GpsInfo);
    void startInit();
    void continueInit();
    void endInit();
    void saveFile();
    GpsBufferReadInitRouteThread *getGpsBufferReadInitRouteThread()
    {
        return gpsBufferReadInitRouteThread;
    }
    ~InitRouteDialog();

private:
    Ui::InitRouteDialog *ui;
    GpsBufferReadInitRouteThread *gpsBufferReadInitRouteThread;
    GpsRingBuffer *gpsRingBuffer;

signals:
    void sendInitSignal(bool signal);
};

#endif // INITROUTEDIALOG_H
