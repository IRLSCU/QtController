#include "GPSProcesser.h"
#include<QDebug>
#include<limits>
CCoordinate GPSProcesser::cCoordinate;

GPSProcesser::GPSProcesser() {

}
GPSProcesser::~GPSProcesser() {

}
void GPSProcesser::initCCoordinate(double gpsheight, double gpslongitude, double gpslatitude) {
	cCoordinate.InitRadarPara(gpsheight, gpslongitude, gpslatitude);
    qDebug() << QStringLiteral("初始化原点成功");
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
