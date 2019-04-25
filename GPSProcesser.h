#pragma once

#ifndef GPS_PROCESSER
#define GPS_PROCESSER

#include "Processer.h"
#include"CoTrans.h"
#include"GpsInfo.h"

class GPSProcesser{

public:
	GPSProcesser();
	~GPSProcesser();
	static void initCCoordinate(double gpsheight, double gpslongitude, double gpslatitude);
    bool initRoute(QList<QPointF>);//初始化gps_route QPointF(longitude,latitude)
    int initStartPoint(double longitude,double latitude);//通过当前位置设定起始点，返回在固定路径中最近的位置,若无则返回0;
    int setNextTargetPoint(int i);
    double startProcess(GpsInfo current,int* target,int* status);//通过当前点返回其离直线的距离，用来纠偏
    GaussGPSData gpsData2Gauss(double gpslongitude, double gpslatitude,int quality);
    GaussGPSData gpsData2Gauss(GpsInfo gpsInfo);
	static CCoordinate cCoordinate;

    Processer processer;
};
#endif // GPS_PROCESSER
