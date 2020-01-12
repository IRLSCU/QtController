#include "ProcessRunNoGPSDialog.h"
#include "ui_ProcessRunDialog.h"

#include<QFileDialog>
#include<QMessageBox>
ProcessRunNoGPSDialog::ProcessRunNoGPSDialog(LocationRingBuffer* locationRingBuffer,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ProcessRunDialog)
{
    ui->setupUi(this);
    this->setWindowTitle(QStringLiteral("start running"));
    PID=new Pid_control;
    processer=new Processer();//需要初始化高斯坐标的终点gps信息，需要加载路径，需要设定初始点。
    //用消费者线程处理locationBuffer
    locationBufferConsumeRunThread=new LocationBufferConsumeRunThread(locationRingBuffer,this);//从ringbuffer中消费GPS坐标
    locationBufferConsumeRunThread->start();
    connect(this,&ProcessRunNoGPSDialog::sendInitSignal,locationBufferConsumeRunThread,&LocationBufferConsumeRunThread::runSignal);//开始消费者线程后，标识是否开始
    connect(ui->startPointChoiceBT,&QPushButton::clicked,locationBufferConsumeRunThread,&LocationBufferConsumeRunThread::setSendStartLocationSignal);//开始消费者线程后，标识是否取一个值为车辆当前坐标
    qRegisterMetaType<LocationPosition>("LocationPosition");
    //若点击界面的重新开始或者继续按钮，消费者线程开始传递信号
//    connect(locationBufferConsumeRunThread,&LocationBufferConsumeRunThread::sendRunLocation,[&](LocationPosition location){
//        processLocation(location);//处理传过来的GPS,对车辆当前位置进行处理
//        this->ui->speed->setText(QString::number(location.speed,'f',2));
//        this->ui->course->setText(QString::number(location.course,'f',2));
        //updateBroswerText(location);//更新显示的text broswer
//    });//同时将数据发往ProcessRunDialog
    connect(locationBufferConsumeRunThread,&LocationBufferConsumeRunThread::sendRunLocation,this,&ProcessRunNoGPSDialog::processLocation);
    connect(locationBufferConsumeRunThread,&LocationBufferConsumeRunThread::sendRunLocation,this,&ProcessRunNoGPSDialog::updateBroswerText);

    connect(locationBufferConsumeRunThread,&LocationBufferConsumeRunThread::sendStartLocation,this,&ProcessRunNoGPSDialog::initStartPointByLocation);//接收从消费者线程传出的当前车辆坐标，直接初始化。

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
    connect(ui->saveAs,&QPushButton::clicked,this,&ProcessRunNoGPSDialog::saveFile);


    //send control order thread setting
#ifdef SENDTOROS
    controlOrderSendThread=new ControlOrderSendToRosThread(this);
    connect(this,&ProcessRunNoGPSDialog::sendRange,controlOrderSendThread,&ControlOrderSendToRosThread::setRange);
    connect(this,&ProcessRunNoGPSDialog::sendSpeed,controlOrderSendThread,&ControlOrderSendToRosThread::setSpeed);
    connect(this,&ProcessRunNoGPSDialog::sendLocationInfo,controlOrderSendThread,&ControlOrderSendToRosThread::setLocationInfo);
    connect(this,&ProcessRunNoGPSDialog::sendDoNothingSignal,controlOrderSendThread,&ControlOrderSendToRosThread::doNothing);
#else
    controlOrderSendThread=new ControlOrderSendThread(this);
    connect(this,&ProcessRunNoGPSDialog::sendRange,controlOrderSendThread,&ControlOrderSendThread::setRange);
    connect(this,&ProcessRunNoGPSDialog::sendSpeed,controlOrderSendThread,&ControlOrderSendThread::setSpeed);
    connect(this,&ProcessRunNoGPSDialog::sendLocationInfo,controlOrderSendThread,&ControlOrderSendThread::setLocationInfo);
    connect(this,&ProcessRunNoGPSDialog::sendDoNothingSignal,controlOrderSendThread,&ControlOrderSendThread::doNothing);
#endif



    connect(ui->start,&QPushButton::clicked,[&](){
        controlOrderSendThread->enableSignal(true);
    });
    connect(ui->pause,&QPushButton::clicked,[&](){
        controlOrderSendThread->enableSignal(false);
    });
    connect(ui->continue_2,&QPushButton::clicked,[&](){
        controlOrderSendThread->enableSignal(true);
    });
    //发送控制指令按钮设置，先把之前打开的线程关闭，再打开
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
    //设置速度与挡位
    this->ui->orderSpeed->setText(QString::number(CONTROLORDER_MAX_SPEED/8));
    this->ui->orderGear->setText(QString::number(CONTROLORDER_GEAR_FORWARD));
    connect(ui->orderInitSettingBT,&QPushButton::clicked,[&](){
        bool speedOk;
        bool gearOk;
        int speed=ui->orderSpeed->text().toInt(&speedOk);
        int gear=ui->orderGear->text().toInt(&gearOk);
        if(speedOk){
            controlOrderSendThread->setSpeed(speed);
        }else{
            controlOrderSendThread->setSpeed(CONTROLORDER_ZERO_SPEED);
            QMessageBox::warning(this, tr("speed parameter error"),
                                       tr("please input integer"));
        }
        if(gearOk){
            if(gear>0){
                controlOrderSendThread->setGear(CONTROLORDER_GEAR_FORWARD);
            }
            else if(gear==0){
                controlOrderSendThread->setGear(CONTROLORDER_GEAR_ZERO);
            }
            else{
                controlOrderSendThread->setGear(CONTROLORDER_GEAR_BACKWARD);
            }
        }else{
            controlOrderSendThread->setGear(CONTROLORDER_GEAR_ZERO);
            QMessageBox::warning(this, tr("gear parameter error"),
                                       tr("please input integer"));
        }
    });
}

ProcessRunNoGPSDialog::~ProcessRunNoGPSDialog()
{
    locationBufferConsumeRunThread->stopImmediately();
    locationBufferConsumeRunThread->wait();

    controlOrderSendThread->stopImmediately();
    controlOrderSendThread->wait();
    delete ui;
}
void ProcessRunNoGPSDialog::processLocation(LocationPosition location){
    if(location.x==0&&location.y==0&&location.z==0){
        //qDebug()<<"do not accept correct ros data";
        emit sendDoNothingSignal(true);
        return;
    }else {
        emit sendDoNothingSignal(false);
    }
    int status;//状态，0未到终点，1到达终点
    int target;//下一个目标点
    //计算获得的偏离程度。r
    GaussGPSData current(location.x,location.y);
    double actualCTE = processer->startProcess(current,&target,&status);
    setNextTargetPoint(target);
    //转化成车辆的转向
    double rangeCrossError=PID->PID_realize(actualCTE);
    double targetDiffAngle=processer->calTargetYawDiff(current);
    double range=calRange(rangeCrossError,targetDiffAngle);
    //todo 车辆速度PID
    //double speed=12;
    emit sendRange((int)range);
    emit sendLocationInfo(location);
    //emit sendSpeed((int)speed);
    ui->rangeCTE->setText(QString::number(actualCTE));
    //ui->speedCTE->setText(QString::number(speed));
    if(status==1){
        //todo 可能需要传到一个线程
        controlOrderSendThread->stopImmediately();
        this->ui->sendCOStatus->setText(QStringLiteral("已经到达终点，断开与车辆连接"));
        qDebug()<<QStringLiteral("已到终点");
    }
}
void ProcessRunNoGPSDialog::updateBroswerText(LocationPosition location){
    ui->textBrowser->insertPlainText(location.toString().append("\n"));
    ui->textBrowser->moveCursor(QTextCursor::End);
}
//使用在主界面中
void ProcessRunNoGPSDialog::copySetInitRouteList(QList<QPointF> route){
    for(int i=0;i<route.size();i++){
        this->routePointList.append(QPointF(route.at(i)));
    }
    processer->initRoute(this->routePointList);
}
void ProcessRunNoGPSDialog::startProcess(){
    this->ui->textBrowser->clear();
    emit sendInitSignal(true);
}
void ProcessRunNoGPSDialog::continueProcess(){
    emit sendInitSignal(true);
}
void ProcessRunNoGPSDialog::endProcess(){
    emit sendInitSignal(false);
}

void ProcessRunNoGPSDialog::saveFile()
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
void ProcessRunNoGPSDialog::initStartPointByLocation(LocationPosition location){
    emit sendStartPointLocationToPaintWidget(QPointF(location.x,location.y));
    initStartPoint(location.x,location.y,location.z);
}
void ProcessRunNoGPSDialog::initStartPoint(double x, double y, double z){
    int target=processer->initStartPoint(GaussGPSData(x,y));
    setNextTargetPoint(target);
    initStartPointToQLable(x,y,z);//QLabel显示
}
void ProcessRunNoGPSDialog::setNextTargetPoint(int i){
    ui->lineEdit->setText(QString::number(i));
    processer->setNextTargetPoint(i);
    emit sendNextTargetPointToPaintWidget(i);
}
void ProcessRunNoGPSDialog::initStartPointToQLable(double longitude, double latitude, double altitude){
    this->ui->lon->setText(QString::number(longitude,'f',10));
    this->ui->lat->setText(QString::number(latitude,'f',10));
    this->ui->alt->setText(QString::number(altitude,'f',10));
}
void ProcessRunNoGPSDialog::readFile()
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
double ProcessRunNoGPSDialog::calRange(double rangeCrossError,double targetDiffAngle){
    double range=rangeCrossError;
    int add=1;
    if(targetDiffAngle<0){
        add=-1;
    }
    if(fabs(targetDiffAngle)>20){
        add*=2000;
    }else{
        add*=4000;
    }
//    qDebug()<<"add=:"<<add;
    return range+add;
}
