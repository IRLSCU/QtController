#include "SocketSettingWidget.h"
#include "ui_SocketSettingWidget.h"
#include "GpsBufferWriteThread.h"

#include<QFileDialog>
#include<QMessageBox>
SocketSettingWidget::SocketSettingWidget(GpsRingBuffer* gpsRingBuffer,LocationRingBuffer*locationRingBuffer,QWidget *parent) :
    QWidget(parent,Qt::Window),
    ui(new Ui::SocketSettingWidget)
{
    ui->setupUi(this);
    socket = new QTcpSocket();
    this->gpsRingBuffer=gpsRingBuffer;

    //for test
    ringBuffer1=new CharRingBuffer;
    gpsBufferWriteThread1=new GpsBufferWriteThread(ringBuffer1,this->gpsRingBuffer,this,"gpsBufferWriteThread1");
    //gpsBufferWriteThread1->start();


    //连接信号槽
    QObject::connect(socket, &QTcpSocket::readyRead, this, &SocketSettingWidget::socket_Read_Data);
    QObject::connect(socket, &QTcpSocket::disconnected, this, &SocketSettingWidget::socket_Disconnected);

    ui->lineEdit_IP->setText("127.0.0.1");
    ui->lineEdit_Port->setText("8765");

    this->locationRingBuffer=locationRingBuffer;
    locationBufferProduceThread=new LocationBufferProduceThread(ringBuffer1,this->locationRingBuffer,this,"locationBufferProduceThread");
    locationBufferProduceThread->start();
}

SocketSettingWidget::~SocketSettingWidget()
{
    gpsBufferWriteThread1->stopImmediately();
    gpsBufferWriteThread1->wait();

    locationBufferProduceThread->stopImmediately();
    locationBufferProduceThread->wait();
    delete gpsBufferWriteThread1;
    delete ringBuffer1;
    delete this->socket;

    delete ui;
}
void SocketSettingWidget::on_pushButton_Connect_clicked()
{
    if(ui->pushButton_Connect->text() == tr("connect"))
    {
        QString IP;
        int port;

        //获取IP地址
        IP = ui->lineEdit_IP->text();
        //获取端口号
        port = ui->lineEdit_Port->text().toInt();

        //取消已有的连接
        socket->abort();
        //连接服务器
        socket->connectToHost(IP, port);

        //等待连接成功
        if(!socket->waitForConnected(30000))
        {
            qDebug() << "Connection failed!";
            return;
        }
        qDebug() << "Connect successfully!";

        //修改按键文字
        ui->pushButton_Connect->setText("disconnect");
    }
    else
    {
        //断开连接
        socket->disconnectFromHost();
        //修改按键文字
        ui->pushButton_Connect->setText("connect");
    }
}
void SocketSettingWidget::on_pushButton_Clear_clicked(){
    ui->textEdit_Recv->clear();
}
void SocketSettingWidget::on_pushButton_Save_clicked(){
    QString path = QFileDialog::getSaveFileName(this,
                                                tr("Open File"),
                                                "./../QtControl/GnnNema",
                                                tr("Text Files(*.txt)"));
    if(!path.isEmpty()) {
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, tr("Write File"),
                                       tr("Cannot open file:\n%1").arg(path));
            return;
        }
        QTextStream out(&file);
        out << ui->textEdit_Recv->toPlainText();
        file.close();
    }
}
void SocketSettingWidget::socket_Read_Data()
{
    QByteArray buffer;
    //读取缓冲区数据
    buffer = socket->readAll();
    if(!buffer.isEmpty())
    {
        foreach (char c, buffer) {
            ringBuffer1->push(c);
        }
        //刷新显示
        ui->textEdit_Recv->insertPlainText(buffer);
        ui->textEdit_Recv->moveCursor(QTextCursor::End);
    }
}

void SocketSettingWidget::socket_Disconnected()
{
    //修改按键文字
    ui->pushButton_Connect->setText("connect");
    qDebug() << "Disconnected!";
}
