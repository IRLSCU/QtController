#ifndef LOCATIONPOSITION_H
#define LOCATIONPOSITION_H

#include<QDebug>
struct LocationPosition
{
    double x;
    double y;
    double z;
    double speed;
    double course;
    double precision;
    unsigned long timestamp;

    LocationPosition(double x=0,double y=0,double z=0,double speed=0,double course=0,double precision=1,unsigned long timestamp=0)
        :x(x),y(y),z(z),speed(speed),course(course),precision(precision),timestamp(timestamp){}
    void printInfo(){
        qDebug()<<toString();
    }
    QString toString(){
        QString content=QObject::tr("%1 %2 %3 %4 %5 %6 %7").arg(x,2,'f',4).arg(y,2,'f',4).arg(z,2,'f',4)
                .arg(speed,2,'f',2).arg(course,2,'f',2).arg(precision,2,'f',2).arg(timestamp);
        return content;
    }
};
#endif // LOCATIONPOSITION_H
