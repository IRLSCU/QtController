/*#include "stdafx.h"*/
//经纬度同直角坐标系之间的转换

#include<QDebug>
#include "CoTrans.h"

#include <math.h>

const qreal   PI = 3.14159265;                          ///<圆周率
const qreal   ErthR0 = 6378.137;                        ///<地球赤道参考半径（千米）//6371.116;
const qreal   EARTH_RADIUS_METER = ErthR0*1000;         ///<地球赤道参考半径（米）
const qreal   EibSiLon = 0.0818191;                     ///<地球第一偏心率

#ifndef DBL_EPSILON
#define DBL_EPSILON      (2.2204460492503131e-6)         ///实型最小值
#endif

//	CCoordinate
//*****************************************
CCoordinate::CCoordinate()
{
    m_fixedScale = 1;
    m_scale = 1;
    m_DspCenter.setX(0);
    m_DspCenter.setY(0);
}
LongLat CCoordinate::GetCenterLL()
{
	m_LLPos.Lon = m_Lon;
	m_LLPos.Lat=m_Lat;
	m_LLPos.Hei = m_Hei;
	return m_LLPos;
}
//初始化中心位置
void CCoordinate::InitRadarPara(qreal height,qreal longitude,qreal latitude)
{
    // longitude 经度坐标，单位：度
	// latitude  纬度坐标，单位：度
	// height 高度 单位：米
    qreal medval,Lat,Lon;

	m_Hei = height;
    m_Lon = longitude;
	m_Lat = latitude;

	Lat = latitude*PI/180.0;
    Lon = longitude*PI/180.0;


	m_latcos=cos(Lat);
	m_latsin=sin(Lat);
    m_longcos=cos(Lon);
    m_longsin=sin(Lon);
	
	medval=EibSiLon*EibSiLon;

	m_Radius=EARTH_RADIUS_METER*sqrt(1-medval*m_latsin*m_latsin);  //地球等效半径

	//	Radar[iRID].Radius=6371137.0*sqrt(1-medval)/(1.0-medval*Radar[iRID].latsin);
	m_RadarHei=m_Radius + m_Hei;   //目标距球心距离
}

//-----------------------------------------------
// pos		待转化点的经、纬度坐标，单位：度
// m_XYPos	转化后的平面直角坐标，	单位：米
//-----------------------------------------------
QPointF CCoordinate::LongLat2XY(const qreal & lon,const qreal & lat,const qreal & hei)
{
    LongLat pos;
	pos.Lon=lon;
	pos.Lat=lat;
	pos.Hei=hei;
    return LongLat2XY(pos);

}

QPointF CCoordinate::LongLat2XY(const qreal & lon,const qreal & lat)
{
    LongLat pos;
	pos.Lon=lon;
	pos.Lat=lat;
	pos.Hei=m_Hei;
    return LongLat2XY(pos);
}


QPointF CCoordinate::LongLat2XY(const LongLat & pos)
{
    qreal Lat,Lon;
    qreal latCos,latSin;
    qreal DeltaLonCos,DeltaLonSin;
    qreal medval;
	
	Lat=pos.Lat*PI/180.0;
	Lon=pos.Lon*PI/180.0;
	latCos=cos(Lat);
	latSin=sin(Lat);

	DeltaLonCos=cos(Lon-m_Lon*PI/180.0);
	DeltaLonSin=sin(Lon-m_Lon*PI/180.0);
	
	medval=1+latSin*m_latsin+latCos*m_latcos*DeltaLonCos;

    m_XYPos.setX((2.0*m_RadarHei*latCos*DeltaLonSin/medval));
    m_XYPos.setY((2.0*m_RadarHei*(latSin*m_latcos-latCos*m_latsin*DeltaLonCos)/medval));
	
	return m_XYPos;
}

LongLat CCoordinate::XY2LongLat(QPointF pos)
{
	//-----------------------------------------------
	// pos		待转化点的平面直角坐标，单位：米
	// m_LLPos	转化后的经纬度坐标，单位：度
	//-----------------------------------------------

    qreal Px,Py;
    qreal templong;
    qreal RHei,RLatCos,RLatSin,RLat,RLon;

    Px = pos.x();
    Py = pos.y();
	
	RHei = m_RadarHei;
	RLat = m_Lat/180.0*PI;
	RLon = m_Lon/180.0*PI;
	RLatCos = m_latcos;
	RLatSin = m_latsin;
	
    templong = RLon;
	
    if(pos.x()==0)
	{
		m_LLPos.Lon=m_Lon;
	}
	else
	{
        templong += atan(4*RHei*Px/((4*RHei*RHei-Px*Px-Py*Py)*RLatCos-4*RHei*Py*RLatSin));
        m_LLPos.Lon=(qreal)(templong*180.0/PI);
	}
	
    templong -= RLon;
    if (pos.x()!=0)
        m_LLPos.Lat=(qreal)(atan(Py*sin(templong)/(Px*RLatCos)+tan(RLat)*cos(templong))*180.0/PI);
	else
        m_LLPos.Lat=(qreal)(m_Lat+180.0/PI*asin(4*RHei*Py/(4*RHei*RHei+Py*Py)));
	
	
	m_LLPos.Hei = m_Hei;
	return m_LLPos;
}
LongLat CCoordinate::XY2LongLat(const qreal &x,const qreal &y)
{
    QPointF xyp;
    xyp.setX(x);
    xyp.setY(y);
    return XY2LongLat(xyp);
}

QPointF CCoordinate::XY2Screen(const qlonglong &x,const qlonglong &y)
{
    QPointF xyp;
    xyp.setX(x);
    xyp.setY(y);
    return XY2Screen(xyp);
}

QPointF CCoordinate::XY2Screen(QPointF pos)
{
    qreal f_tempX,f_tempY;
    f_tempX = (pos.x() -m_DspCenter.x())*m_fixedScale * m_scale;
    f_tempY = (pos.y()-m_DspCenter.y())*m_fixedScale * m_scale;
    m_screen.setX((long)(f_tempX+0.5));
    m_screen.setY(-(long)(f_tempY+0.5));
    return m_screen;
}

QPointF CCoordinate::Screen2XY(QPointF pos)
{
    qreal f_tempX,f_tempY;
    f_tempX =(qreal)(( pos.x() / m_fixedScale / m_scale)+ m_DspCenter.x());
    f_tempY =(qreal)((-pos.y() / m_fixedScale / m_scale) + m_DspCenter.y());

    m_XYPos.setX(f_tempX);
    m_XYPos.setY(f_tempY);
    return m_XYPos;
}

QPointF CCoordinate::Screen2XY(const qlonglong &x,const qlonglong &y)
{
    QPointF xyp;
    xyp.setX(x);
    xyp.setY(y);

    return Screen2XY(xyp);
}

QPointF CCoordinate::LongLat2Screen(LongLat pos)
{
    m_XYPos = LongLat2XY(pos);
    //qDebug("LongLat2Screen:(:%.10f,%.10f)",m_XYPos.x(),m_XYPos.y());
    m_screen = XY2Screen(m_XYPos);
    return m_screen;
}

LongLat CCoordinate::Screen2LongLat(QPointF pos)
{
    m_XYPos = Screen2XY(pos);
    //qDebug("Screen2LongLat:(%.10f,%.10f)",m_XYPos.x(),m_XYPos.y());
    m_LLPos = XY2LongLat(m_XYPos);
    return m_LLPos;
}

LongLat CCoordinate::Screen2LongLat(const qlonglong &x,const qlonglong &y)
{
    QPointF xyp;
    xyp.setX(x);
    xyp.setY(y);
    return Screen2LongLat(xyp);
}
