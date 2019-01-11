#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "SerialPortDialog.h"
#include "PaintWidget.h"
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
    void openSerialDialog();
    void openFile();
    QAction* setSerialAction;   //打开端口设置
    QAction* initRouteAction;   //初始化路径
    QAction* startRunningAction;//延预计路径行驶
    QAction* loadGPSDataAction; //加载GPS数据到界面，并且作为初始化路径
    QAction* setScaleAction;    //设置比例尺

    PaintWidget* paintWidget;

signals:
    void sendQPointToPaintWidget(QPointF&);
};

#endif // MAINWINDOW_H
