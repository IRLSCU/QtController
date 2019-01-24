#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "WriteGPSBufferThread.h"
#include <QAction>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>
#include <QToolBar>
#include <QDebug>
#include <QFileDialog>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle(tr("Main Window"));


    setCentralWidget(paintWidget=new PaintWidget());
    connect(this,&MainWindow::sendQPointToPaintWidget,paintWidget,&PaintWidget::acceptQPoint);

    setSerialAction=new QAction(tr("串口设置"),this);
    setSerialAction->setShortcuts(QKeySequence::Open);
    setSerialAction->setStatusTip(tr("打开设置串口界面"));
    connect(setSerialAction, &QAction::triggered, this, &MainWindow::openSerialDialog);

    initRouteAction=new QAction(tr("初始化路径"),this);
    initRouteAction->setShortcuts(QKeySequence::Open);
    initRouteAction->setStatusTip(tr("打开初始化路径界面"));
    connect(initRouteAction, &QAction::triggered, this, &MainWindow::open);

    startRunningAction=new QAction(tr("开始行驶"),this);
    startRunningAction->setShortcuts(QKeySequence::Open);
    startRunningAction->setStatusTip(tr("打开开始行驶界面"));
    connect(startRunningAction, &QAction::triggered, this, &MainWindow::open);

    loadGPSDataAction=new QAction(tr("加载GPS"),this);
    loadGPSDataAction->setShortcuts(QKeySequence::Open);
    loadGPSDataAction->setStatusTip(tr("打开加载GPS界面"));
    connect(loadGPSDataAction, &QAction::triggered, this, &MainWindow::openFile);

    setScaleAction=new QAction(tr("比例尺"),this);
    setScaleAction->setShortcuts(QKeySequence::Open);
    setScaleAction->setStatusTip(tr("打开比例尺界面"));
    connect(setScaleAction, &QAction::triggered, this, &MainWindow::open);

    QToolBar *toolBar = addToolBar(tr("&File"));

    toolBar->addAction(setSerialAction);
    toolBar->addAction(initRouteAction);
    toolBar->addAction(startRunningAction);
    toolBar->addAction(loadGPSDataAction);
    toolBar->addAction(setScaleAction);

    statusBar();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::open(){
    QDialog* dialog=new QDialog;
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowTitle(tr("Hello, dialog!"));
    dialog->show();
}
void MainWindow::openFile()
{
    QString path = QFileDialog::getOpenFileName(this,
                                                tr("Open File"),
                                                "./../QtControl/route",
                                                tr("Text Files(*.txt)"));
    if(!path.isEmpty()) {
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QMessageBox::warning(this, tr("Read File"),
                                 tr("Cannot open file:\n%1").arg(path));
            return;
        }
        QTextStream in(&file);
        while (!in.atEnd()) {
            qreal x=0;
            qreal y=0;
            in>>x>>y;
            qDebug("%.10f", x);
            qDebug("%.10f", y);
            QPointF point(x,y);
            sendQPointToPaintWidget(point);//signal
        }

        file.close();
    } else {
        QMessageBox::warning(this, tr("Path"),
                             tr("You did not select any file."));
    }
}
void MainWindow::openSerialDialog(){
    SerialPortDialog* dialog=new SerialPortDialog(this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowTitle(tr("Hello, dialog!"));
    dialog->show();

}

void MainWindow::print(const QString& title){
    qDebug()<<title;
}
