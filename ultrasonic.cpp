#include "ultrasonic.h"
#include <QLatin1Char>

Ultrasonic::Ultrasonic(QObject *parent) : QObject(parent)
{
    m_serialPort = new QSerialPort();
    //this->connect(this, SIGNAL(sendStringInfo(QString)), this, SLOT(calcDistance()));
}
Ultrasonic::~Ultrasonic()
{
    if (m_serialPort->isOpen())
    {
        m_serialPort->close();
    }
    delete m_serialPort;
}



void Ultrasonic::openPort(){
    if(m_serialPort->isOpen())//如果串口已经打开了 先给他关闭了
    {
        m_serialPort->clear();
        m_serialPort->close();
    }
//    m_serialPort->setPortName(m_PortNameComboBox->currentText());//当前选择的串口名字
    portName = FileOperation::readFile("./../QtControl/ultrasonicPortNameConfig.txt").trimmed();
    qDebug()<<portName;
    m_serialPort->setPortName(portName);
    if(!m_serialPort->open(QIODevice::ReadWrite))//用ReadWrite 的模式尝试打开串口
    {
        qDebug()<<"打开失败!";
        return;
    }
    qDebug()<<"串口打开成功!";

    m_serialPort->setBaudRate(QSerialPort::Baud9600,QSerialPort::AllDirections);//设置波特率和读写方向
    m_serialPort->setDataBits(QSerialPort::Data8);      //数据位为8位
    m_serialPort->setFlowControl(QSerialPort::NoFlowControl);//无流控制
    m_serialPort->setParity(QSerialPort::NoParity); //无校验位
    m_serialPort->setStopBits(QSerialPort::OneStop); //一位停止位

    connect(m_serialPort,SIGNAL(readyRead()),this,SLOT(calcDistance()));
}
QString Ultrasonic::receiveInfo(){
    //qDebug()<<"receive data begain : ";
    QByteArray state = m_serialPort->readAll();
    stringInfo = QString(state.toHex());
    //qDebug()<<"receive info:"<<stringInfo;
    emit sendStringInfo(stringInfo);
    return stringInfo;
}
void Ultrasonic::closePort(){
    if(m_serialPort->isOpen()){
        m_serialPort->close();

        qDebug()<<"port closed";
    }
}

int Ultrasonic::calcDistance(){
    receiveInfo();
    //qDebug()<<"stringInfo bytes:1111111111111111111"<<stringInfo;
    d_byte_str = stringInfo.mid(8,4);
    //qDebug()<<"distance_str bytes:"<<d_byte_str;
    bool pass = checkData(stringInfo);
    if(pass){
        distance = d_byte_str.toInt(nullptr ,16);
        emit sendDistance(distance<=THRESHOLD,distance);
    }
    else
        qDebug()<<"data error!!!!:";
    //qDebug()<<"distance: "<<distance;
    return distance;
}

int Ultrasonic::getDistance(){
    return distance;
}
int Ultrasonic::hexString2Int(QString s)
{
    int i;
    int n = 0;
    int len =2;
    if(s.length()<len) return 0; //解决崩毁，指针溢出问题

    for (i=0;i<len;++i)
        {
            if (s.at(i) > '9')
            {
                n = 16 * n + (10 + s.at(i).toLatin1() - 'A');
            }
            else
            {
                n = 16 * n +( s.at(i).toLatin1() - '0');
            }
        }
        return n;
}
bool Ultrasonic::checkData(QString s){
    QStringList list;
    QString check_byte_str = s.mid(12,2);
    list.append(s.mid(0,2));
    list.append(s.mid(2,2));
    list.append(s.mid(4,2));
    list.append(s.mid(6,2));
    list.append(s.mid(8,2));
    list.append(s.mid(10,2));
    int checkNum = check_byte_str.toInt(nullptr,16);
    int totalSum =0;
    for(int i = 0;i<list.length();i++){
        totalSum += list.at(i).toInt(nullptr,16);
    }
    totalSum = QString("%1").arg(totalSum,2,16,QLatin1Char('0')).mid(1,2).toInt(nullptr,16);  // 保留四位，不足补零
    //qDebug()<<"checkNum"<<checkNum;
    //qDebug()<<"totalSumHex"<<totalSum;
    if(checkNum == totalSum){
        return true;
    }
    return false;
}
