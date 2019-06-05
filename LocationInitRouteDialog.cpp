#include "LocationInitRouteDialog.h"
#include "ui_locationinitroutedialog.h"
#include <QFileDialog>
#include <QMessageBox>
LocationInitRouteDialog::LocationInitRouteDialog(LocationRingBuffer*locationRingBuffer, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LocationInitRouteDialog)
{
    ui->setupUi(this);
    this->setWindowTitle(QStringLiteral("初始化路径"));
    locationConsumThread=new LocationBufferConsumInitRouteThread(locationRingBuffer,this);
    locationConsumThread->start();

    connect(this,&LocationInitRouteDialog::sendInitSignal,locationConsumThread,&LocationBufferConsumInitRouteThread::initSignal);
    qRegisterMetaType<GpsInfo>("GpsInfo");
    //将数据显示与获取Gps信息绑定
    connect(locationConsumThread,&LocationBufferConsumInitRouteThread::sendInitLocationInfo,this,&LocationInitRouteDialog::updateBroswerText);


    //控制读线程开始与暂停
    connect(ui->restartBT,&QPushButton::clicked,this,&LocationInitRouteDialog::startInit);
    connect(ui->pauseBT,&QPushButton::clicked,this,&LocationInitRouteDialog::endInit);
    connect(ui->continueBT,&QPushButton::clicked,this,&LocationInitRouteDialog::continueInit);

    //另存为
    connect(ui->saveBT,&QPushButton::clicked,this,&LocationInitRouteDialog::saveFile);

}

LocationInitRouteDialog::~LocationInitRouteDialog()
{
    locationConsumThread->stopImmediately();
    locationConsumThread->wait();
    delete ui;
}

void LocationInitRouteDialog::updateBroswerText(QString location){
    ui->textBrowser->insertPlainText(location.append('\n'));
    ui->textBrowser->moveCursor(QTextCursor::End);
}

void LocationInitRouteDialog::startInit(){
    this->ui->textBrowser->clear();
    emit sendInitSignal(true);
}
void LocationInitRouteDialog::continueInit(){
    emit sendInitSignal(true);
}
void LocationInitRouteDialog::endInit(){
    emit sendInitSignal(false);
}

void LocationInitRouteDialog::saveFile()
{
    QString path = QFileDialog::getSaveFileName(this,
                                                tr("Open File"),
                                                "./../QtControl/routeXYZ",
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
