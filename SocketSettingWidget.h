#ifndef SOCKETSETTINGWIDGET_H
#define SOCKETSETTINGWIDGET_H

#include "GpsBufferWriteThread.h"
#include "LocationBufferProduceThread.h"
#include <QWidget>
#include <QTcpSocket>

namespace Ui {
class SocketSettingWidget;
}
//Socket设置,通过Tcp获取GPS的定位信息
class SocketSettingWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SocketSettingWidget(GpsRingBuffer* gpsRingBuffer,LocationRingBuffer*,QWidget *parent = 0);
    ~SocketSettingWidget();

private slots:
    void socket_Read_Data();
    void socket_Disconnected();
    void on_pushButton_Connect_clicked();
    void on_pushButton_Clear_clicked();
    void on_pushButton_Save_clicked();
private:
    Ui::SocketSettingWidget *ui;
    QTcpSocket *socket;
    GpsRingBuffer* gpsRingBuffer;
    GpsBufferWriteThread* gpsBufferWriteThread1;
    CharRingBuffer* ringBuffer1;

    LocationBufferProduceThread* locationBufferProduceThread;//for test
    LocationRingBuffer* locationRingBuffer;
};

#endif // SOCKETSETTINGWIDGET_H
