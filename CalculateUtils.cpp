#include "CalculateUtils.h"
#include <QDebug>
#include <math.h>
CalculateUtils::CalculateUtils()
{
}
double CalculateUtils::distanceTwoPoint(QPointF p1, QPointF p2)
{
    double x1 = p1.x();
    double y1 = p1.y();
    double x2 = p2.x();
    double y2 = p2.y();
    return pow(pow(x2 - x1, 2) + pow(y2 - y1, 2), 0.5);
}
//逆时针旋转,center旋转中心,roratepoint旋转点,angle旋转角度
QPointF CalculateUtils::roratePoint(QPointF center, QPointF roratePoint, double angle)
{
    double x = roratePoint.x(), y = roratePoint.y(); //旋转的点
    double dx = center.x(), dy = center.y();         //被绕着旋转的点
    double xx = (x - dx) * cos(angle * PI / 180) - (y - dy) * sin(angle * PI / 180) + dx;
    double yy = (y - dy) * cos(angle * PI / 180) + (x - dx) * sin(angle * PI / 180) + dy;
    QPointF targePoint(xx, yy);
    return targePoint;
}
/*
 * 以center为起点表示当前车辆行驶方向,根据上一个点和当前定位点，粗略地表示车辆的行驶方向
 */
QPointF CalculateUtils::calCoursePoint(QPointF center, QPointF last, double length)
{
    QPointF coursePoint;
    double distance = CalculateUtils::distanceTwoPoint(center, last);
    double ratio = length / distance;
    coursePoint.setX((center.x() - last.x()) * ratio + center.x());
    coursePoint.setY((center.y() - last.y()) * ratio + center.y());
    return coursePoint;
}
/*
 * 根据order中转向角度，绘制转向方向
 * 以center为中心，将coursePoint转动转向角度，仅表示程度值
 * 从模型上公式上推导，该转向程度值和实际转向值(角速度)都是线性的
 */
QPointF CalculateUtils::calRangePoint(QPointF center, QPointF coursePoint, int range)
{
    QPointF rangePoint;
    //qDebug()<<"range"<<range;
    //-32768~+32767  线性映射 -60~+60°
    double angle = -range / (32768.0) * 60;
    rangePoint = CalculateUtils::roratePoint(center, coursePoint, angle);
    rangePoint.setX(rangePoint.x());
    rangePoint.setY(rangePoint.y());
    qDebug() << "angle:" << angle << " range:" << range;
    return rangePoint;
}
