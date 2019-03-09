#include "ProcessRunDialog.h"
#include "ui_ProcessRunDialog.h"

#include<QFileDialog>
#include<QMessageBox>
ProcessRunDialog::ProcessRunDialog(GpsRingBuffer* gpsRingBuffer,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ProcessRunDialog)
{
    ui->setupUi(this);
    this->setWindowTitle(tr("start running"));
    gpsProcesser=new GPSProcesser();//需要初始化高斯坐标的终点gps信息，需要加载路径，需要设定初始点。

    gpsBufferConsumeRunThread=new GpsBufferConsumeRunThread(gpsRingBuffer,this);
    gpsBufferConsumeRunThread->start();
    connect(this,&ProcessRunDialog::sendInitSignal,gpsBufferConsumeRunThread,&GpsBufferConsumeRunThread::runSignal);//开始消费者线程后，标识是否开始
    connect(ui->startPointChoiceBT,&QPushButton::clicked,gpsBufferConsumeRunThread,&GpsBufferConsumeRunThread::setSendStartGpsInfoSignal);//开始消费者线程后，标识是否取一个值为车辆当前坐标
    qRegisterMetaType<GpsInfo>("GpsInfo");
    connect(gpsBufferConsumeRunThread,&GpsBufferConsumeRunThread::sendRunGpsInfo,this,&ProcessRunDialog::updateBroswerText);
    connect(gpsBufferConsumeRunThread,&GpsBufferConsumeRunThread::sendStartGpsInfo,this,&ProcessRunDialog::initStartPointByGpsInfo);//接收从消费者线程传出的当前车辆坐标，直接初始化。

    //控制读线程开始与暂停
    connect(ui->start,&QPushButton::clicked,this,&ProcessRunDialog::startProcess);
    connect(ui->pause,&QPushButton::clicked,this,&ProcessRunDialog::endProcess);
    connect(ui->continue_2,&QPushButton::clicked,this,&ProcessRunDialog::continueProcess);

    //另存为
    connect(ui->saveAs,&QPushButton::clicked,this,&ProcessRunDialog::saveFile);
}

ProcessRunDialog::~ProcessRunDialog()
{
    gpsBufferConsumeRunThread->stopImmediately();
    gpsBufferConsumeRunThread->wait();
    delete ui;
}

void ProcessRunDialog::updateBroswerText(GpsInfo gpsInfo){
    ui->textBrowser->insertPlainText(gpsInfo.toString());
    ui->textBrowser->moveCursor(QTextCursor::End);
}
void ProcessRunDialog::copySetInitRouteList(QList<QPointF> route){
    for(int i=0;i<route.size();i++){
        this->routePointList.append(QPointF(route.at(i)));
    }
    initGpsProcessRoute(routePointList);
    initCoordinateOriginPoint(routePointList.at(0));
}
void ProcessRunDialog::initGpsProcessRoute(QList<QPointF> route){
    gpsProcesser->initRoute(route);
}
void ProcessRunDialog::initCoordinateOriginPoint(QPointF originPoint){//初始化高斯坐标系坐标原点
    gpsProcesser->initCCoordinate(500,originPoint.x(),originPoint.y());
}
void ProcessRunDialog::startProcess(){
    this->ui->textBrowser->clear();
    emit sendInitSignal(true);
}
void ProcessRunDialog::continueProcess(){
    emit sendInitSignal(true);
}
void ProcessRunDialog::endProcess(){
    emit sendInitSignal(false);
}

void ProcessRunDialog::saveFile()
{
    QString path = QFileDialog::getSaveFileName(this,
                                                tr("Open File"),
                                                "./../QtControl/route",
                                                tr("Text Files(*.txt)"));
    if(!path.isEmpty()) {
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, tr("Write File"),
                                       tr("Cannot open file:\n%1").arg(path));
            return;
        }
        QTextStream out(&file);
        out << ui->textBrowser->toPlainText();
        file.close();
    } else {
        QMessageBox::warning(this, tr("Path"),
                             tr("You did not select any file."));
    }
}
void ProcessRunDialog::initStartPointByGpsInfo(GpsInfo gps){
    initStartPoint(gps.longitude,gps.latitude,gps.altitude);
}
void ProcessRunDialog::initStartPoint(double longitude, double latitude, double altitude){
    gpsProcesser->initStartPoint(GaussGPSData(longitude,latitude));
    initStartPointToQLable(longitude,latitude,altitude);
}
void ProcessRunDialog::initStartPointToQLable(double longitude, double latitude, double altitude){
    this->ui->lon->setText(QString::number(longitude,'f',10));
    this->ui->lat->setText(QString::number(latitude,'f',10));
    this->ui->alt->setText(QString::number(altitude,'f',10));
}
