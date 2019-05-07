#include "Processer.h"
#include<QDebug>
#include<limits>
#include<qmath.h>
Processer::Processer() {

}
Processer::~Processer() {

}
/**
 * @brief Processer::initRoute
 * @param route 高斯类型数据
 * @return
 */
bool Processer::initRoute(QList<QPointF> route){
    for(int i=0;i<route.size();i++){
        gps_route.push_back(GaussGPSData(route.at(i).x(),route.at(i).y()));
    }
    sum_gps_point=gps_route.size();
    if(sum_gps_point==0){
        qDebug()<<QStringLiteral("初始化路径失败，可能未加载路径，或路径无数据点");
        return false;
    }else{
        qDebug()<<QStringLiteral("初始化路径成功，共有点数（个）:")<<sum_gps_point;
        return true;
    }
}
int Processer::setNextTargetPoint(int i){
    i=i%gps_route.size();
    next_route_gps=gps_route[i];
    if(i==0){
        qDebug()<<QStringLiteral("gps processer setNextTargetPoint is setting 1");
        current_point_count=0;
        current_route_gps=gps_route[1];
    }
    if(i!=0){
        current_route_gps=gps_route[i-1];
        current_point_count=i-1;
    }
    return i;
}
int Processer::initStartPoint(GaussGPSData current) {
	current_gps = current;
    for (unsigned int i = 0; i < gps_route.size();i++) {
        if (isInArea(gps_route[i], current, IN_DISTANCE)) {
            if (i + 1 == sum_gps_point ) {
                //std::cout << "初始化车辆起始位置失败，车辆已在道路终点\n";
                qDebug()<<QStringLiteral("初始化车辆起始位置失败，车辆已在道路终点");
                return 0;
            }
            //std::cout << "初始化车辆起始位置成功，车辆位置靠近第" << i << "个路径的GPS高斯点\n";
            qDebug()<<QStringLiteral("初始化车辆起始位置成功，车辆位置靠近第") << i << QStringLiteral("个路径的GPS高斯点");
            return i;
        }
    }
    //std::cout << "初始化起始车辆位置为路径起点，车辆偏离道路太远\n";
    qDebug()<<QStringLiteral("初始化起始车辆位置为路径起点，车辆偏离道路太远");
    return 0;
}
//double GPSProcesser::startProcess(GaussGPSData current) {
//	last_gps = current_gps;
//	current_gps = current;
//	int flag = 0;//记录是否移动了靠近点
//	for (int i = current_point_count; i < sum_gps_point; i++) {
//		if (isInArea(current, gps_route[i], IN_DISTANCE)) {//判断是否到了下一个GPSRoute上的点
//			if (i == sum_gps_point-1) {
//				std::cout << "车辆已在道路终点\n";
//				return -1;
//			}
//			current_point_count = i;
//			current_route_gps = gps_route[i];
//			next_route_gps = gps_route[i + 1];
//			std::cout << getPointDistance(current, gps_route[i]) <<"车辆已经过第" << i << "个路径的GPS高斯点\n";
//			flag = 1;
//			break;
//		}
//	}
//	if(flag==0)
//		std::cout << getPointDistance(current,next_route_gps) <<"车辆位置未经过任意一高斯点" << IN_DISTANCE << "米，上一个经过的高斯点为第"<<current_point_count<<"个\n";
//	double distance = calPointFromLineDistance(current, Line(gps_route[current_point_count], gps_route[current_point_count + 1]));
//	double director = calPointFromLineDirector(current, Line(gps_route[current_point_count], gps_route[current_point_count + 1]));
//	return distance*director;
//}
/**
 * @brief Processer::startProcess
 * @param current 传入当前高斯坐标坐标
 * @param target 传出目标位置在route的位置
 * @param status 传出状态(1：到达终点；0：未达终点)
 * @return
 */
double Processer::startProcess(GaussGPSData current,int* target,int* status) {
	last_gps = current_gps;
	current_gps = current;
	Line corssLine = getCorssLine(Line(current_route_gps, next_route_gps), next_route_gps);
	if (pointsInSameSide(current,current_route_gps , corssLine)) {
        //std::cout << "车辆未经过任意一高斯点(过垂线),"<<"上一个经过的高斯点为第" << current_point_count << "个\n";
        //qDebug()<< QStringLiteral("车辆未经过任意一高斯点(过垂线),上一个经过的高斯点为:")<< current_point_count;

	}else {
		current_point_count += 1;
        if (current_point_count >= sum_gps_point-1) {
            *status=1;
            current_point_count=0;
            qDebug()<< QStringLiteral("车辆已在道路终点,重置当前点为起始点。");
        }else{
            *status=0;
        }
        current_route_gps = gps_route[current_point_count];
        next_route_gps = gps_route[current_point_count + 1];
        qDebug()<< QStringLiteral("车辆已经过第")<<current_point_count<<QStringLiteral("个高斯点，车辆目标为第") << current_point_count+1 << QStringLiteral("个路径的GPS高斯点");
	}
    *target = current_point_count+1;
    double distance = calPointFromLineDistance(current, Line(current_route_gps, next_route_gps));
    double director = calPointFromLineDirector(current, Line(current_route_gps, next_route_gps));
	return distance * director;
}
double Processer::startProcess2(GaussGPSData current) {
	last_gps = current_gps;
	current_gps = current;
	int flag = 0;//记录是否移动了靠近点
	for (int i = current_point_count; i < sum_gps_point; i++) {
		if (isInArea(current, gps_route[i], IN_DISTANCE)) {//判断是否到了下一个GPSRoute上的点，方法一：通过是否靠近下个点IN_DISTANCE距离。
			if (i >= sum_gps_point - 1) {
                qDebug()<< QStringLiteral("车辆已在道路终点");
				return -1;
			}
			current_point_count = i;
			current_route_gps = gps_route[i];
			next_route_gps = gps_route[i + 1];
            qDebug()<< QStringLiteral("通过方法一(距离)，车辆经过第") << i << QStringLiteral("个路径的GPS高斯点");
			flag = 1;
			break;
		}
	}
	if (flag == 0) {//车辆未靠近任意一点.方法二：当前GPS位置和上一次获取的GPS点，是否均在目标点和目标点的下一个点的连接线一侧上，若在两侧，则说明应该将当前点设置为目标点，目标点设计为当前目标点的下一个。
		if (current_point_count + 2 < sum_gps_point) {//若已在当前路径下的最后第二个点，则无法通过方法二判断。
			bool inSameSide = pointsInSameSide(last_gps, current_gps, Line(next_route_gps, gps_route[current_point_count + 2]));//当前GPS位置和上一次获取的GPS点，是否在目标点和目标点的下一个点的连接线一侧上
			if (!inSameSide) {//如果不在一条直线上，则车辆可以说明已经经过目标点。
				current_point_count += 1;
				current_route_gps = gps_route[current_point_count];
				next_route_gps = gps_route[current_point_count + 1];
			}
			else {
                 qDebug()<< QStringLiteral("未经过任意高斯点，车辆未靠近任意一高斯点") << IN_DISTANCE << QStringLiteral("米，上一个靠近的高斯点为第" )<< current_point_count << QStringLiteral("个");
			}
		}
		else {
             qDebug()<< QStringLiteral("正在向终点前进，无法通过方法二(点同侧)判断是否到终点。");
		}
	}
	double distance = calPointFromLineDistance(current, Line(gps_route[current_point_count], gps_route[current_point_count + 1]));
	double director = calPointFromLineDirector(current, Line(gps_route[current_point_count], gps_route[current_point_count + 1]));
	return distance * director;
}
double Processer::getPointDistance(GaussGPSData start, GaussGPSData end) {
	double distance = sqrt(pow((start.m_x - end.m_x), 2.0) + pow(start.m_y - end.m_y, 2.0));
	return distance;
}

double Processer::getLineDistance(Line line) {
	return getPointDistance(line.m_start, line.m_end);
}

bool Processer::isInArea(const GaussGPSData source, const GaussGPSData destination, const double range) {//判断是否到达下个点
	return getPointDistance(source, destination) <= range;
}

double Processer::calPointFromLineDistance(const GaussGPSData point, const Line line) {
	//Ax+By+C=0;
	//|Ax0+By0+C|/sqrt(A^2+B^2)
	double A, B, C;
	A = line.m_end.m_y - line.m_start.m_y;
	B = line.m_start.m_x - line.m_end.m_x;
	C = line.m_start.m_y*line.m_end.m_x - line.m_start.m_x*line.m_end.m_y;
	assert((A != 0 || B != 0));
    double distance = fabs(A*point.m_x + B * point.m_y + C) / sqrt(A*A + B * B);
	return distance;
}
/*
	向量AB=line，点C在向量AB的左边(-1)，右边（1），在向量上0
	转化成向量AB与向量AC的向量积
*/
double Processer::calPointFromLineDirector(const GaussGPSData point, const Line line) {
	Line AC(line.m_start,point);
	Line AB = line;				
	double AB_X = AB.m_end.m_x - AB.m_start.m_x;
	double AB_Y = AB.m_end.m_y - AB.m_start.m_y;
	double AC_X = AC.m_end.m_x - AC.m_start.m_x;
	double AC_Y = AC.m_end.m_y - AC.m_start.m_y;
	double director =  AB_Y*AC_X- AB_X * AC_Y;
	if (director < 0)
		director = -1;
	else if (director > 0)
		director = 1;
	else
		director = 0;
	return director;
}

double Processer::pointProduct(Line line1, Line line2) {
	double pointProduct = (line1.m_end.m_x - line1.m_start.m_x)*(line2.m_end.m_x - line2.m_start.m_x)
		+ (line1.m_end.m_y - line1.m_start.m_y)*(line2.m_end.m_y - line2.m_start.m_y);
	return pointProduct;
}
bool Processer::pointsInSameSide(GaussGPSData point1, GaussGPSData point2, Line line) {
	return calPointFromLineDirector(point1, line) == calPointFromLineDirector(point2, line);
}
/*
	垂直于line，并且过point点
*/
Line Processer::getCorssLine(Line line, GaussGPSData point) {
	if (line.m_start.m_x == line.m_end.m_x) {
		if (line.m_start.m_y == line.m_end.m_y) {
			return Line(GaussGPSData(0,0,FIXED_SOLUTION), GaussGPSData(1,0,FIXED_SOLUTION));
		}
		return Line(GaussGPSData(0.0, point.m_y, FIXED_SOLUTION), GaussGPSData(1.0, point.m_y, FIXED_SOLUTION));
	}
	double k = (line.m_end.m_y - line.m_start.m_y) / (line.m_end.m_x - line.m_start.m_x);
	GaussGPSData otherPoint(0, point.m_y + (1/k) * point.m_x,FIXED_SOLUTION);
	return Line(point, otherPoint);
}
/*
	line1 到line2的转向角
*/
double Processer::calLinesAngle(Line line1, Line line2) {
	double k1 = calLineK(line1);
	double k2 = calLineK(line2);
	return calLinesAngle(k1, k2);
}
/*
k1到k2的转向角
*/
double Processer::calLinesAngle(double k1, double k2) {
	double tan_k = 0; //直线夹角正切值
	double lines_arctan;//直线斜率的反正切值
	tan_k = (k2 - k1) / (1 + k1*k2); //求直线夹角的公式
	lines_arctan = atan(tan_k)* 180.0 / 3.1415926;
	return lines_arctan;
}
double Processer::calLineK(Line line) {
	double temp = line.m_end.m_x - line.m_start.m_x;
	if (temp == 0) {
        return std::numeric_limits<double>::max();
	}
	else {
		return (line.m_end.m_y - line.m_start.m_y) / temp;
	}
}
