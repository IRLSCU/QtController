#ifndef GPSINFO_H
#define GPSINFO_H
#include<QDebug>
struct GpsInfo
{
    double longitude;//经度
    double latitude;//纬度
    double height;//高度
    double speed;//速度
    GpsInfo(double lon=0,double lat=0,double hei=0,double speed=0):longitude(lon),latitude(lat),height(hei),speed(speed) {}
    void printInfo(){
        qDebug("longitude:%.10f,latitude:%.10f,height:%.10f,speed:%.10f",longitude,latitude,height,speed);
    }
};
Q_DECLARE_METATYPE(GpsInfo)
#endif // GPSINFO_H
