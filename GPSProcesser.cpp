#include "GPSProcesser.h"
#include<QDebug>
#include<limits>
CCoordinate GPSProcesser::cCoordinate;

GPSProcesser::GPSProcesser() {
    initCCoordinate();
}
GPSProcesser::~GPSProcesser() {

}
void GPSProcesser::initCCoordinate() {
    QFile file("./../QtControl/CoordinateConf.txt");
    if(file.open(QIODevice::ReadOnly | QIODevice::Text)){
        QByteArray t ;
        while(!file.atEnd())
        {
            t += file.readLine();
        }
        QList<QByteArray> sl =t.split(' ');
        if(sl.size()==3){
            bool ok1;
            bool ok2;
            bool ok3;
            double lon=sl.at(0).toDouble(&ok1);
            double lat=sl.at(1).toDouble(&ok2);
            double hei=sl.at(2).toDouble(&ok3);
            if(ok1&&ok2&&ok3){
                cCoordinate.InitRadarPara(hei, lon, lat);
                qDebug() << QStringLiteral("初始化原点成功");
            }else{
                qDebug()<<"load coordinateConf data fail";
            }
         }else{
            qDebug()<<"load coordinateConf data fail";
        }
    }else{
        qDebug()<<"open coordinateConf failed";
    }
    file.close();
}
bool GPSProcesser::initRoute(QList<QPointF> gps){
    QList<QPointF> route;
    for(int i=0;i<gps.size();i++){
        QPointF temp=cCoordinate.LongLat2XY(gps.at(i).x(),gps.at(i).y());
        route.push_back(temp);
    }
    return processer.initRoute(route);
}
int GPSProcesser::setNextTargetPoint(int i){
    return processer.setNextTargetPoint(i);
}

int GPSProcesser::initStartPoint(double longitude, double latitude) {
    GaussGPSData current=gpsData2Gauss(longitude,latitude,FIXED_SOLUTION);
    return processer.initStartPoint(current);
}
/**
 * @brief GPSProcesser::startProcess
 * @param current 传入当前gps坐标
 * @param target 传出目标点坐标
 * @param status 传出状态(1：到达终点；0：未达终点)
 * @return
 */
double GPSProcesser::startProcess(GpsInfo gpsInfo,int* target,int* status) {
    GaussGPSData current=gpsData2Gauss(gpsInfo);
    return processer.startProcess(current,target,status);
}
GaussGPSData GPSProcesser::gpsData2Gauss(double gpslongitude, double gpslatitude,int quality=FIXED_SOLUTION) {
    QPointF xyPoint = cCoordinate.LongLat2XY(gpslongitude, gpslatitude);
    return GaussGPSData(xyPoint.x(), xyPoint.y(), quality);
}
GaussGPSData GPSProcesser::gpsData2Gauss(GpsInfo gpsInfo) {
    QPointF xyPoint = cCoordinate.LongLat2XY(gpsInfo.longitude, gpsInfo.latitude);
    return GaussGPSData(xyPoint.x(), xyPoint.y(), gpsInfo.quality);
}
