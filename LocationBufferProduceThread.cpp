#include "LocationBufferProduceThread.h"

#include <QDebug>
#include <string.h>
LocationBufferProduceThread::LocationBufferProduceThread(
    CharRingBuffer *ringBuffer,
    LocationRingBuffer *loactionRingBuffer,
    QObject *parent = 0,
    QString name = tr("")) : QThread(parent)
{
    this->charRingBuffer = ringBuffer;
    this->loactionRingBuffer = loactionRingBuffer;
    this->name = name;
    NewParser = new NmeaPres();
    if (NewParser->NmeaInitParsers() == false)
    {
        qDebug() << "init nmea parser error";
        delete NewParser;
    }
    initCCoordinate();
}
void LocationBufferProduceThread::run()
{ //将char缓冲区中的数据拼接成gpsInfo类型数据，并存入环形缓冲区
    m_isCanRun = true;
    qDebug() << name << " LocationBufferProduceThread has started";
    QString stringBuffer;
    char receive_char;
    while (true)
    {
        if (charRingBuffer->pop(receive_char))
        {
            stringBuffer.append(receive_char);
            if (receive_char == 10)
            {
                std::string gpsData;
                if (NewParser->isValidSentenceChecksum(stringBuffer.toStdString(), gpsData))
                {
                    NewParser->ParseNmea0183Sentence(gpsData);
                    GlobalGpsStruct globalGps;
                    NewParser->getGpsGlobalStruct(globalGps);
                    double lon = globalGps.fLongitude;
                    double lat = globalGps.fLatitude;
                    QPointF point = ordinate.LongLat2XY(lon, lat);
                    if (isGGA(stringBuffer))
                        loactionRingBuffer->push(LocationPosition(point.x(), point.y()));
                }
                //qDebug()<<stringBuffer;
                stringBuffer.clear();
            }
        }
        else
        {
            this->msleep(LOCATIONBUFFERPRODUCETHREAD_BOLCKTIME); //阻塞否则cpu占用太高
        }
        QMutexLocker locker(&m_lock);
        if (!m_isCanRun) //在每次循环判断是否可以运行，如果不行就退出循环
        {
            return;
        }
        else
        {
            locker.unlock();
        }
    }
}
LocationBufferProduceThread::~LocationBufferProduceThread()
{
    qDebug() << name << " has been destoried";
    delete NewParser;
}
void LocationBufferProduceThread::stopImmediately()
{
    QMutexLocker locker(&m_lock);
    m_isCanRun = false;
    locker.unlock();
}
bool LocationBufferProduceThread::isGGA(QString s)
{
    if (s.size() < 8)
        return false;
    QString hex = s.mid(1, 5);
    if (hex.compare("GPGGA") != 0)
        return false;
    //    for (int i = 0; i<s.size(); i++) {
    //        if (s[i] == '*') return true;
    //    }
    //    return false;
    return true;
}
void LocationBufferProduceThread::initCCoordinate()
{
    QFile file("./../QtControl/CoordinateConf.txt");
    if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QByteArray t;
        while (!file.atEnd())
        {
            t += file.readLine();
        }
        QList<QByteArray> sl = t.split(' ');
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
                ordinate.InitRadarPara(hei, lon, lat);
                qDebug() << QStringLiteral("初始化原点成功");
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
