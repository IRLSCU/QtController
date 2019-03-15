#include "ProcessRunDialog.h"
#include "ui_ProcessRunDialog.h"

#include<QFileDialog>
#include<QMessageBox>
ProcessRunDialog::ProcessRunDialog(GpsRingBuffer* gpsRingBuffer,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ProcessRunDialog)
{
    ui->setupUi(this);
    this->setWindowTitle(QStringLiteral("start running"));
    PID=new Pid_control;
    gpsProcesser=new GPSProcesser();//需要初始化高斯坐标的终点gps信息，需要加载路径，需要设定初始点。
    gpsBufferConsumeRunThread=new GpsBufferConsumeRunThread(gpsRingBuffer,this);
    gpsBufferConsumeRunThread->start();
    connect(this,&ProcessRunDialog::sendInitSignal,gpsBufferConsumeRunThread,&GpsBufferConsumeRunThread::runSignal);//开始消费者线程后，标识是否开始
    connect(ui->startPointChoiceBT,&QPushButton::clicked,gpsBufferConsumeRunThread,&GpsBufferConsumeRunThread::setSendStartGpsInfoSignal);//开始消费者线程后，标识是否取一个值为车辆当前坐标
    qRegisterMetaType<GpsInfo>("GpsInfo");
    //若点击界面的重新开始或者继续按钮，消费者线程开始传递信号
    connect(gpsBufferConsumeRunThread,&GpsBufferConsumeRunThread::sendRunGpsInfo,[&](GpsInfo gpsInfo){
        processGPS(gpsInfo);//处理传过来的GPS,对车辆当前位置进行处理
        updateBroswerText(gpsInfo);//更新显示的text broswer
    });//同时将数据发往ProcessRunDialog
    connect(gpsBufferConsumeRunThread,&GpsBufferConsumeRunThread::sendStartGpsInfo,this,&ProcessRunDialog::initStartPointByGpsInfo);//接收从消费者线程传出的当前车辆坐标，直接初始化。

    //设置下一个目标点
    connect(ui->confirmNextPointBT,&QPushButton::clicked,[&](){
        bool ok=true;
        int target=ui->lineEdit->text().toInt(&ok,10);
        if(!ok){
            QMessageBox::warning(this, tr("error input"),
                                 tr("please input numbers"));
        }else{
            this->setNextTargetPoint(target);
        }
    });
    //设置PID
    connect(ui->ConfirmPID,&QPushButton::clicked,[&](){
        bool PisOk=true;
        bool IisOk=true;
        bool DisOk=true;
        double P=this->ui->PLineEdit->text().toDouble(&PisOk);
        double I=this->ui->ILineEdit->text().toDouble(&IisOk);
        double D=this->ui->DLineEdit->text().toDouble(&DisOk);
        if(PisOk&&IisOk&&DisOk){
            PID->PID_init(P,I,D,0,0,0.1);
            QFile file("./../QtControl/PidConfig.txt");
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            file.write(QObject::tr("%1 %2 %3").arg(P,0,'f').arg(I,0,'f').arg(D,0,'f').toUtf8());
            file.close();
        }else{
            QMessageBox::warning(this, tr("error input"),tr("please input numbers"));
        }
    });
    //从目录中加载PID数据
    connect(ui->loadPID,&QPushButton::clicked,[&](){
        readFile();
    });
    //控制读线程开始与暂停
    connect(ui->start,&QPushButton::clicked,[&](){
        this->startProcess();
        this->ui->receiveStatus->setText(QStringLiteral("已开始"));
    });
    connect(ui->pause,&QPushButton::clicked,[&](){
        this->endProcess();
        this->ui->receiveStatus->setText(QStringLiteral("已暂停"));
    });
    connect(ui->continue_2,&QPushButton::clicked,[&](){
        this->continueProcess();
        this->ui->receiveStatus->setText(QStringLiteral("已继续"));
    });
    //另存为
    connect(ui->saveAs,&QPushButton::clicked,this,&ProcessRunDialog::saveFile);


    //send control order thread setting
    controlOrderSendThread=new ControlOrderSendThread(this);
//    connect(this,&ProcessRunDialog::sendRange,controlOrderSendThread,&ControlOrderSendThread::setRange,Qt::BlockingQueuedConnection);
//    connect(this,&ProcessRunDialog::sendSpeed,controlOrderSendThread,&ControlOrderSendThread::setSpeed,Qt::BlockingQueuedConnection);
    connect(this,&ProcessRunDialog::sendRange,controlOrderSendThread,&ControlOrderSendThread::setRange);
    connect(this,&ProcessRunDialog::sendSpeed,controlOrderSendThread,&ControlOrderSendThread::setSpeed);

    connect(ui->start,&QPushButton::clicked,[&](){
        controlOrderSendThread->enableSignal(true);
    });
    connect(ui->pause,&QPushButton::clicked,[&](){
        controlOrderSendThread->enableSignal(false);
    });
    connect(ui->continue_2,&QPushButton::clicked,[&](){
        controlOrderSendThread->enableSignal(true);
    });
    //先把之前打开的线程关闭，再打开
    connect(ui->startSendingCO,&QPushButton::clicked,[&](){
       this->ui->sendCOStatus->setText(QStringLiteral("与车辆连接"));
       controlOrderSendThread->stopImmediately();
       controlOrderSendThread->wait();
       controlOrderSendThread->start();
    });
    connect(ui->stopSendingCO,&QPushButton::clicked,[&](){
       this->ui->sendCOStatus->setText(QStringLiteral("断开与车辆连接"));
       controlOrderSendThread->stopImmediately();
    });
}

ProcessRunDialog::~ProcessRunDialog()
{
    gpsBufferConsumeRunThread->stopImmediately();
    gpsBufferConsumeRunThread->wait();

    controlOrderSendThread->stopImmediately();
    controlOrderSendThread->wait();
    delete ui;
}
void ProcessRunDialog::processGPS(GpsInfo gpsInfo){
    QPointF current=gpsProcesser->cCoordinate.LongLat2XY(gpsInfo.longitude,gpsInfo.latitude);
    int status;//状态，0未到终点，1到达终点
    int target;//下一个目标点
    //gps转XY坐标，计算获得的偏离程度。
    double actualCTE = gpsProcesser->startProcess(GaussGPSData(current.x(),current.y()),&target,&status);
    setNextTargetPoint(target);
    //转化成车辆的转向
    double range=PID->PID_realize(actualCTE);
    //todo 车辆速度PID
    double speed=12;
    emit sendRange((int)range);
    emit sendSpeed((int)speed);
    ui->rangeCTE->setText(QString::number(actualCTE));
    //ui->speedCTE->setText(QString::number(speed));
    if(status==1){
        //todo 可能需要传到一个线程
        controlOrderSendThread->stopImmediately();
        this->ui->sendCOStatus->setText(QStringLiteral("已经到达终点，断开与车辆连接"));
        qDebug()<<QStringLiteral("已到终点");
    }
}
void ProcessRunDialog::updateBroswerText(GpsInfo gpsInfo){
    ui->textBrowser->insertPlainText(gpsInfo.toString());
    ui->textBrowser->moveCursor(QTextCursor::End);
}
void ProcessRunDialog::copySetInitRouteList(QList<QPointF> route){
    for(int i=0;i<route.size();i++){
        this->routePointList.append(QPointF(route.at(i)));
    }
    initCoordinateOriginPoint(routePointList.at(0));
    initGpsProcessRoute(routePointList);

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
    emit sendStartPointGPSToPaintWidget(QPointF(gps.longitude,gps.latitude));
    initStartPoint(gps.longitude,gps.latitude,gps.altitude);
}
void ProcessRunDialog::initStartPoint(double longitude, double latitude, double altitude){
    int target=gpsProcesser->initStartPoint(GaussGPSData(longitude,latitude));
    setNextTargetPoint(target);
    initStartPointToQLable(longitude,latitude,altitude);//QLabel显示
}
void ProcessRunDialog::setNextTargetPoint(int i){
    ui->lineEdit->setText(QString::number(i));
    gpsProcesser->setNextTargetPoint(i);
    emit sendNextTargetPointToPaintWidget(i);
}
void ProcessRunDialog::initStartPointToQLable(double longitude, double latitude, double altitude){
    this->ui->lon->setText(QString::number(longitude,'f',10));
    this->ui->lat->setText(QString::number(latitude,'f',10));
    this->ui->alt->setText(QString::number(altitude,'f',10));
}
void ProcessRunDialog::readFile()
{
    QFile file;
    QString f = QFileDialog::getOpenFileName(this, QStringLiteral("选择文件"),
                                             "./../QtControl",
                                             tr("Text Files(*.txt)"));
    file.setFileName(f);
    if(file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QByteArray t ;
        while(!file.atEnd())
        {
            t += file.readLine();
        }
        QList<QByteArray> sl =t.split(' ');
        if(sl.size()<3){
            QMessageBox::warning(this, tr("load PID parameter error"),
                                       tr("need more parameter"));

        }else{
            ui->PLineEdit->setText(sl.at(0));
            ui->ILineEdit->setText(sl.at(1));
            ui->DLineEdit->setText(sl.at(2));
        }
        file.close();
    }
}
