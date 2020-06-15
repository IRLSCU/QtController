#ifndef CALCULATEUTILS_H
#define CALCULATEUTILS_H

#define PI   3.141592654
#include <QPointF>

class CalculateUtils
{
public:
    CalculateUtils();
    static double distanceTwoPoint(QPointF,QPointF);
    static QPointF roratePoint(QPointF center,QPointF roratePoint,double angle);
    static QPointF calCoursePoint(QPointF center,QPointF last, double length);
    static QPointF calRangePoint(QPointF center,QPointF coursePoint,int range);

};

#endif // CALCULATEUTILS_H
