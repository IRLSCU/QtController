
#include <QAction>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>
#include <QToolBar>
#include <QDebug>
#include <QFileDialog>

#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "InitRouteDialog.h"
#include "RouteSparseDialog.h"
#include "ProcessRunDialog.h"
#include "SocketSettingWidget.h"
#include "TinyCarSerialPortDialog.h"

#include "ProcessRunNoGPSDialog.h"
#include "LocationInitRouteDialog.h"
#include "RosSettingDialog.h"
#include "BaseTool.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    // 主窗口UI设置
    ui->setupUi(this);
    setWindowTitle(QStringLiteral("主窗口"));
    // 加载中心点坐标
    loadCenterGPS();
    // 初始化画布坐标系
    initOrdinate();
    // 对应缓冲区初始化函数，注意在这里进行内存分配
    // todo添加析构函数对其进行消除
    gpsRingBuffer = new GpsRingBuffer();
    locationRingBuffer = new LocationRingBuffer();
    // 创建画布
    paintWidget = new PaintWidget(this);

    this->setCentralWidget(paintWidget);
    // 连接信号与画布槽函数
    connect(this, &MainWindow::sendQPointToPaintWidget, paintWidget, &PaintWidget::acceptQPoint);
    // 设置串口动作
    setSerialAction = new QAction(QStringLiteral("串口设置"), this);
    setSerialAction->setShortcuts(QKeySequence::Open);
    setSerialAction->setStatusTip(QStringLiteral("打开设置串口界面"));
    connect(setSerialAction, &QAction::triggered, this, &MainWindow::openSerialDialog);

    setSocketAction = new QAction(QStringLiteral("Socket设置"), this);
    setSocketAction->setShortcuts(QKeySequence::Open);
    setSocketAction->setStatusTip(QStringLiteral("打开Socket设置界面"));
    connect(setSocketAction, &QAction::triggered, this, &MainWindow::openSocketDialog);

    initRouteAction = new QAction(QStringLiteral("初始化路径"), this);
    initRouteAction->setShortcuts(QKeySequence::Open);
    initRouteAction->setStatusTip(QStringLiteral("打开初始化路径界面"));
    connect(initRouteAction, &QAction::triggered, this, &MainWindow::openInitRouteDialog);

    initXYZRouteAction = new QAction(QStringLiteral("初始化路径(XYZ)"), this);
    initXYZRouteAction->setShortcuts(QKeySequence::Open);
    initXYZRouteAction->setStatusTip(QStringLiteral("打开初始化路径界面"));
    connect(initXYZRouteAction, &QAction::triggered, this, &MainWindow::openLocationInitRouteDialog);

    startRunningAction = new QAction(QStringLiteral("开始行驶"), this);
    startRunningAction->setShortcuts(QKeySequence::Open);
    startRunningAction->setStatusTip(QStringLiteral("打开开始行驶界面"));
    connect(startRunningAction, &QAction::triggered, this, &MainWindow::openProcessRun);

    startRunningXYZAction = new QAction(QStringLiteral("开始行驶(XYZ)"), this);
    startRunningXYZAction->setShortcuts(QKeySequence::Open);
    startRunningXYZAction->setStatusTip(QStringLiteral("打开开始行驶界面"));
    connect(startRunningXYZAction, &QAction::triggered, this, &MainWindow::openProcessRunNoGPS);

    loadGPSDataAction = new QAction(QStringLiteral("加载路径"), this);
    loadGPSDataAction->setShortcuts(QKeySequence::Open);
    loadGPSDataAction->setStatusTip(QStringLiteral("打开加载路径界面"));
    connect(loadGPSDataAction, &QAction::triggered, this, &MainWindow::openFile);

    loadXYZDataAction = new QAction(QStringLiteral("加载路径(XYZ)"), this);
    loadXYZDataAction->setShortcuts(QKeySequence::Open);
    loadXYZDataAction->setStatusTip(QStringLiteral("打开加载路径界面"));
    connect(loadXYZDataAction, &QAction::triggered, this, &MainWindow::openXYZFile);

    routeSparseAction = new QAction(QStringLiteral("稀疏路径"), this);
    routeSparseAction->setShortcuts(QKeySequence::Open);
    routeSparseAction->setStatusTip(QStringLiteral("打开稀疏路径界面"));
    connect(routeSparseAction, &QAction::triggered, this, &MainWindow::openRouteSparseDialog);

    setScaleAction = new QAction(QStringLiteral("比例尺"), this);
    setScaleAction->setShortcuts(QKeySequence::Open);
    setScaleAction->setStatusTip(QStringLiteral("打开比例尺界面"));
    // 注意这里监听鼠标滑轮事件
    connect(setScaleAction, &QAction::triggered, this, &MainWindow::open);
    // 设置串口模型
    setTinyCarComAction = new QAction(QStringLiteral("小车串口设置"), this);
    setTinyCarComAction->setShortcuts(QKeySequence::Open);
    setTinyCarComAction->setStatusTip(QStringLiteral("小车串口设置"));
    connect(setTinyCarComAction, &QAction::triggered, this, &MainWindow::openTinyCarSerialDialog);

    //xyz
    setRosAction = new QAction(QStringLiteral("Ros设置"), this);
    setRosAction->setShortcuts(QKeySequence::Open);
    setRosAction->setStatusTip(QStringLiteral("打开Ros设置界面"));
    connect(setRosAction, &QAction::triggered, this, &MainWindow::openRosDialog);
    // 设置主窗口条形界面栏目
    QToolBar *toolBar = addToolBar(tr("&File"));
    // 主要条控制调整
    toolBar->addAction(setSerialAction);
    toolBar->addAction(setSocketAction);
    toolBar->addAction(initRouteAction);
    toolBar->addAction(startRunningAction);
    toolBar->addAction(loadGPSDataAction);
    toolBar->addAction(routeSparseAction);
    //toolBar->addAction(setScaleAction);
    toolBar->addAction(setTinyCarComAction);

    toolBar->addAction(startRunningXYZAction);
    toolBar->addAction(initXYZRouteAction);
    toolBar->addAction(loadXYZDataAction);
    toolBar->addAction(setRosAction);

    statusBar();
}

MainWindow::~MainWindow()
{
    delete ui;
    DELETE_OBJECT(gpsRingBuffer)
    DELETE_OBJECT(locationRingBuffer)
}
void MainWindow::open()
{
    QDialog *dialog = new QDialog(this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowTitle(tr("Hello, dialog!"));
    dialog->show();
}
void MainWindow::openSocketDialog()
{
    SocketSettingWidget *widget = new SocketSettingWidget(gpsRingBuffer, locationRingBuffer, this);
    widget->setAttribute(Qt::WA_DeleteOnClose);
    widget->setWindowTitle(QStringLiteral("打开Socket设置"));
    widget->show();
}
void MainWindow::openProcessRun()
{
    if (gpsRouteList.size() == 0)
    {
        QMessageBox::warning(this, tr("Empty Route"),
                             tr("please load route firstly"));
        return;
    }
    ProcessRunDialog *dialog = new ProcessRunDialog(gpsRingBuffer, this);
    dialog->copySetInitRouteList(gpsRouteList);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowTitle(tr("Process Run"));
    dialog->show();
    // 画图信号和槽
    connect(dialog->getGpsBufferConsumeRunThread(), &GpsBufferConsumeRunThread::sendGpsInfo, [&](QPointF point)
            {
                QPointF temp = ordinate.LongLat2XY(point.x(), point.y());
                paintWidget->addQPoint(temp);
            });
    // 连接画图信号和槽
    connect(dialog, &ProcessRunDialog::sendStartPointGPSToPaintWidget, [&](QPointF gpsInfo)
            {
                QPointF temp = ordinate.LongLat2XY(gpsInfo.x(), gpsInfo.y());
                qDebug() << gpsInfo.x() << gpsInfo.y();
                paintWidget->paintStartPoint(temp);
            });
    // 设置目标点槽函数连接
    connect(dialog, &ProcessRunDialog::sendNextTargetPointToPaintWidget, paintWidget, &PaintWidget::paintTargetPoint);
}
void MainWindow::openProcessRunNoGPS()
{
    if (locationRouteList.size() == 0)
    {
        QMessageBox::warning(this, tr("Empty Route"),
                             tr("please load route firstly"));
        return;
    }
    ProcessRunNoGPSDialog *dialog = new ProcessRunNoGPSDialog(locationRingBuffer, this);
    //for test 从文件读GPS转化成xy
    for (int i = 0; i < gpsRouteList.size(); i++)
    {
        locationRouteList.push_back(ordinate.LongLat2XY(gpsRouteList.at(i).x(), gpsRouteList.at(i).y()));
    }
    dialog->copySetInitRouteList(locationRouteList);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowTitle(tr("Process Run(Not GPS)"));
    dialog->show();
    connect(dialog->getLocationBufferConsumeRunThread(), &LocationBufferConsumeRunThread::sendLocation, paintWidget, &PaintWidget::addQPoint);
    connect(dialog, &ProcessRunNoGPSDialog::sendStartPointLocationToPaintWidget, paintWidget, &PaintWidget::paintStartPoint);
    connect(dialog, &ProcessRunNoGPSDialog::sendNextTargetPointToPaintWidget, paintWidget, &PaintWidget::paintTargetPoint);
    connect(dialog, &ProcessRunNoGPSDialog::sendCurrentRangeToPaintWidget, paintWidget, &PaintWidget::setRange);
}
void MainWindow::openInitRouteDialog()
{
    InitRouteDialog *initRouteDialog = new InitRouteDialog(gpsRingBuffer, this);
    initRouteDialog->setAttribute(Qt::WA_DeleteOnClose);
    initRouteDialog->show();
    connect(initRouteDialog->getGpsBufferReadInitRouteThread(), &GpsBufferReadInitRouteThread::sendGpsInfo, [&](QPointF point)
            {
                QPointF temp = ordinate.LongLat2XY(point.x(), point.y());
                paintWidget->acceptQPoint(temp);
            });
}
void MainWindow::openLocationInitRouteDialog()
{
    LocationInitRouteDialog *initRouteDialog = new LocationInitRouteDialog(locationRingBuffer, this);
    initRouteDialog->setAttribute(Qt::WA_DeleteOnClose);
    initRouteDialog->show();
    connect(initRouteDialog->getLocationBufferConsumInitRouteThread(), &LocationBufferConsumInitRouteThread::sendLocationPointInfo, [&](QPointF point)
            { paintWidget->acceptQPoint(point); });
}

void MainWindow::openRouteSparseDialog()
{
    RouteSparseDialog *routeSparseDialog = new RouteSparseDialog(this);
    routeSparseDialog->setAttribute(Qt::WA_DeleteOnClose);
    routeSparseDialog->show();
}
void MainWindow::openFile()
{
    // 打开配置文件位置
    QString path = QFileDialog::getOpenFileName(this,
                                                tr("Open File"),
                                                "./route");
    if (!path.isEmpty())
    {
        gpsRouteList.clear();
        paintWidget->clear();
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QMessageBox::warning(this, tr("Read File"),
                                 tr("Cannot open file:\n%1").arg(path));
            return;
        }
        // 读取文件流
        QTextStream in(&file);
        while (!in.atEnd())
        {
            qreal x = 0;
            qreal y = 0;
            qreal o = 0;
            //分别为经度、纬度、质量、时分秒、年月日、高度、速度、航向，后面几个绘图用不着。
            in >> x >> y >> o >> o >> o >> o >> o >> o;
            if (x == 0 && y == 0)
                continue;
            //经纬度转化成XY
            gpsRouteList.append(QPointF(x, y));
            QPointF point = ordinate.LongLat2XY(x, y);
            // 进行画图
            sendQPointToPaintWidget(point); //signal
        }
        file.close();
    }
    else
    {
        QMessageBox::warning(this, tr("Path"),
                             tr("You did not select any file."));
    }
}
void MainWindow::openXYZFile()
{

    QString path = QFileDialog::getOpenFileName(this,
                                                tr("Open File"),
                                                "routeXYZ");
    if (!path.isEmpty())
    {
        locationRouteList.clear();
        paintWidget->clear();
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QMessageBox::warning(this, tr("Read File"),
                                 tr("Cannot open file:\n%1").arg(path));
            return;
        }
        QTextStream in(&file);
        while (!in.atEnd())
        {
            QString str = in.readLine();
            if (str.length() < 10)
                continue;
            LocationPosition location = LocationPosition::stringToLocation(str);
            QPointF pointF(location.x, location.y);
            locationRouteList.append(pointF);
            sendQPointToPaintWidget(pointF); //signal
        }
        file.close();
    }
    else
    {
        QMessageBox::warning(this, tr("Path"),
                             tr("You did not select any file."));
    }
}

void MainWindow::openSerialDialog()
{
    SerialPortDialog *dialog = new SerialPortDialog(this, gpsRingBuffer);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->show();
}
void MainWindow::openTinyCarSerialDialog()
{
    TinyCarSerialPortDialog *dialog = new TinyCarSerialPortDialog();
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->show();
}

void MainWindow::openRosDialog()
{
    RosSettingDialog *dialog = new RosSettingDialog(locationRingBuffer, this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->show();
}

void MainWindow::print(const QString &title)
{
    qDebug() << title;
}
void MainWindow::initOrdinate()
{
    ordinate.InitRadarPara(centerGPS.altitude, centerGPS.longitude, centerGPS.latitude);
}
void MainWindow::loadCenterGPS()
{
    // 打开配置文件,示例为：103.9585726017   30.7838652900
    QFile file("Conf/CoordinateConf.txt");
    // 进行文件读取
    if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QByteArray t;
        while (!file.atEnd())
        {
            t += file.readLine();
        }
        QList<QByteArray> sl = t.split(' ');
        // 检查是XYZ坐标系还是GPS坐标系
        if (sl.size() == 3)
        {
            bool ok1;
            bool ok2;
            bool ok3;
            double lon = sl.at(0).toDouble(&ok1);
            double lat = sl.at(1).toDouble(&ok2);
            double hei = sl.at(2).toDouble(&ok3);
            if (ok1 && ok2 && ok3)
            {
                centerGPS.longitude = lon;
                centerGPS.latitude = lat;
                centerGPS.altitude = hei;
            }
            else
            {
                qDebug() << "load coordinateConf data fail";
            }
        }
        else
        {
            qDebug() << "load coordinateConf data fail";
        }
    }
    else
    {
        qDebug() << "open coordinateConf failed";
    }
    file.close();
}
