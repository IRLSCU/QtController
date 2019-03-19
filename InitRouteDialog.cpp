#include "InitRouteDialog.h"
#include "ui_InitRouteDialog.h"

#include<QFileDialog>
#include<QMessageBox>
InitRouteDialog::InitRouteDialog(GpsRingBuffer* gpsRingBuffer, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::InitRouteDialog)
{
    ui->setupUi(this);
    this->setWindowTitle(QStringLiteral("初始化路径"));
    this->ui->speed->setText("0");
    this->ui->course->setText("0");
    gpsBufferReadInitRouteThread=new GpsBufferReadInitRouteThread(gpsRingBuffer,this);
    gpsBufferReadInitRouteThread->start();

    connect(this,&InitRouteDialog::sendInitSignal,gpsBufferReadInitRouteThread,&GpsBufferReadInitRouteThread::initSignal);
    qRegisterMetaType<GpsInfo>("GpsInfo");
    //将数据显示与获取Gps信息绑定
    connect(gpsBufferReadInitRouteThread,&GpsBufferReadInitRouteThread::sendInitGpsInfo,[&](GpsInfo gpsInfo){
        updateBroswerText(gpsInfo);
        this->ui->speed->setText(QString::number(gpsInfo.speed,'f',2));
        this->ui->course->setText(QString::number(gpsInfo.course,'f',2));
    });

    //控制读线程开始与暂停
    connect(ui->start,&QPushButton::clicked,this,&InitRouteDialog::startInit);
    connect(ui->pause,&QPushButton::clicked,this,&InitRouteDialog::endInit);
    connect(ui->continue_2,&QPushButton::clicked,this,&InitRouteDialog::continueInit);

    //另存为
    connect(ui->saveAs,&QPushButton::clicked,this,&InitRouteDialog::saveFile);
}

InitRouteDialog::~InitRouteDialog()
{
    gpsBufferReadInitRouteThread->stopImmediately();
    gpsBufferReadInitRouteThread->wait();
    delete ui;
}

void InitRouteDialog::updateBroswerText(GpsInfo gpsInfo){
    ui->textBrowser->insertPlainText(gpsInfo.toString());
    ui->textBrowser->moveCursor(QTextCursor::End);
}

void InitRouteDialog::startInit(){
    this->ui->textBrowser->clear();
    emit sendInitSignal(true);
}
void InitRouteDialog::continueInit(){
    emit sendInitSignal(true);
}
void InitRouteDialog::endInit(){
    emit sendInitSignal(false);
}

void InitRouteDialog::saveFile()
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

