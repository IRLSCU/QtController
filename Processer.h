#pragma once

#ifndef PROCESSER
#define PROCESSER
#define IN_DISTANCE 3
#define FIXED_SOLUTION 4
#include<assert.h>
#include<vector>
#include<string.h>
#include<fstream>
#include<iostream>
#include<QList>
#include<QPointF>

struct GaussGPSData//高斯坐标系下数据
{
	double m_x;//经度
	double m_y;//维度
	double m_quality;//质量

    GaussGPSData() {}
    GaussGPSData(double x, double y, double quality=FIXED_SOLUTION) :m_x(x), m_y(y), m_quality(quality) {}
	void setGpsData(double x, double y, double quality){
		m_x = x;
		m_y = y;
		m_quality = quality;
	}
};
struct Line {
	GaussGPSData m_start;
	GaussGPSData m_end;

	Line(GaussGPSData start, GaussGPSData end) {
		m_start = start;
		m_end = end;
	}
};

class Processer{

public:
    Processer();
    ~Processer();

    bool initRoute(QList<QPointF>);
    int initStartPoint(GaussGPSData current);//通过当前位置设定起始点，返回在固定路径中最近的位置,若无则返回0;
    int setNextTargetPoint(int i);
    double startProcess(GaussGPSData current,int* target,int* status);//通过当前点返回其离直线的距离，用来纠偏
	double startProcess2(GaussGPSData current);//通过当前点返回其离直线的距离，用来纠偏

private:
	static double getPointDistance(const GaussGPSData, const GaussGPSData);
	static double getLineDistance(const Line);
	static double pointProduct(const Line, const Line);
	static bool isInArea(const GaussGPSData source, const GaussGPSData destination, const double range);//判断是否到达下个点
	static double calPointFromLineDistance(const GaussGPSData point, const Line line);
	static double calPointFromLineDirector(const GaussGPSData point, const Line line);
	static bool pointsInSameSide(GaussGPSData point1, GaussGPSData point2, Line line);
	static Line getCorssLine(Line line, GaussGPSData point);
	static double calLinesAngle(Line line1, Line line2);
	static double calLineK(Line);
    static double calLinesAngle(double k1, double k2);//通过斜率求

	std::vector<GaussGPSData> gps_route;//固定路径的路径
	GaussGPSData current_gps;//当前点
	GaussGPSData last_gps;//上一个GPS点
	GaussGPSData current_route_gps;//距离路径上最近的GPS点
	GaussGPSData next_route_gps;//目标点
	int current_point_count;//vector上的位置
	int sum_gps_point;//gps取点总和
};
#endif // PROCESSER
