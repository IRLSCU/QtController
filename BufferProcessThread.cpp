#include "BufferProcessThread.h"

#include <QDebug>
#include <string.h>
BufferProcessThread::BufferProcessThread(RingBuffer<QChar, 20480>* ringBuffer, QObject *parent = 0, QString name=tr("")):QThread(parent){
    this->ringBuffer=ringBuffer;
    this->name=name;
    NewParser = new NmeaPres();
    if (NewParser->NmeaInitParsers() == false){
        qDebug()<<"init nmea parser error";
        delete NewParser;
    }
}
void BufferProcessThread::run(){
    m_isCanRun = true;
    qDebug()<<name<<"BufferProcessThread has started";
    QString stringBuffer;
    QChar receive_char;
    while(true){
        if (ringBuffer->pop(receive_char)) {
            stringBuffer.append(receive_char);
            if (receive_char == 10) {
                std::string gpsData;
                if(haveStartAndIsGGA(stringBuffer)&&NewParser->isValidSentenceChecksum(stringBuffer.toStdString(), gpsData)){
                    NewParser->ParseNmea0183Sentence(gpsData);
                    GlobalGpsStruct globalGps;
                    NewParser->getGpsGlobalStruct(globalGps);
                }
                qDebug()<<stringBuffer;
                stringBuffer.clear();
            }
        }
        QMutexLocker locker(&m_lock);
        if(!m_isCanRun)//在每次循环判断是否可以运行，如果不行就退出循环
        {
            return;
        }
    }
}
BufferProcessThread::~BufferProcessThread(){
    qDebug()<<name<<" is being destory";
    delete NewParser;
}
void BufferProcessThread::stopImmediately()
{
    QMutexLocker locker(&m_lock);
    m_isCanRun = false;
}
bool BufferProcessThread::haveStartAndIsGGA(QString s) {
    if (s.size() < 8) return false;
    QString hex=s.mid(1, 5);
    if (hex.compare("GPGGA")!=0)
        return false;
    for (int i = 0; i<s.size(); i++) {
        if (s[i] == '*') return true;
    }
    return false;
}
