#ifndef COTRANS_H
#define COTRANS_H

#include<QPointF>
//***************************
// 坐标转换  结构、类
//***************************

//经度-纬度-高度 Coordinate
struct LongLat
{
    qreal Lon;		//Longitude 度
    qreal Lat;		//Latitude	度
    qreal Hei;	//Height 海拔高度 米

    LongLat(){

    }
    LongLat(qreal lon,qreal lat,qreal hei=0):Lon(lon),Lat(lat),Hei(hei){

    }
};

//class CCoordinate_API CCoordinate  
class  CCoordinate  
{
public:
	CCoordinate();
    ~CCoordinate(){}
public:
    qreal	m_fixedScale;		//固定放大倍数
    qreal	m_scale;			//屏幕显示放大倍数
    QPointF	m_DspCenter;		//显示中心
private:

    //斜距-方位角-高度 coordinate
    qreal m_Hei;	//Height	海拔高度
    qreal m_Lon;	//Longitude 经度 度
    qreal m_Lat;	//Latitude	纬度 度

    qreal m_longsin,m_longcos,m_latsin,m_latcos;	//经纬度的三角函数
    qreal m_Radius;		//地球等效半径
    qreal m_RadarHei;	//目标距球心距离
//attribute
private:
	LongLat m_LLPos;		//绝对坐标，经纬度，1/10^4 度
    QPointF	m_XYPos;		//相对于中心点的平面坐标，米
    QPointF	m_screen;
	

public:
    void	InitRadarPara(qreal radarheight,qreal radarlongitude,qreal radarlatitude);
	LongLat GetCenterLL();
	
    QPointF	LongLat2XY(const LongLat & pos);
    QPointF LongLat2XY(const qreal & lon,const qreal & lat,const qreal & hei);
    QPointF LongLat2XY(const qreal & lon,const qreal & lat);

    LongLat	XY2LongLat(QPointF pos);
    LongLat XY2LongLat(const qreal &x,const qreal &y);

    QPointF	XY2Screen(QPointF pos);
    QPointF XY2Screen(const qlonglong &x,const qlonglong &y);
    QPointF	Screen2XY(QPointF pos);
    QPointF	Screen2XY(const qlonglong &x,const qlonglong &y);
    QPointF	LongLat2Screen(LongLat pos);
    LongLat Screen2LongLat(QPointF pos);
    LongLat Screen2LongLat(const qlonglong &x,const qlonglong &y);

};

#endif
