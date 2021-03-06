﻿#ifndef PAINTWIDGET_H
#define PAINTWIDGET_H

#define PAINTWIDGET_DELTA_MAX  18
#define PAINTWIDGET_DELTA_MIN  0
#include"CoTrans.h"
#include "GpsInfo.h"
#include <QPoint>
#include<QLabel>
#include <QWidget>
#include <QList>
#include <QGestureEvent>
#include <QPanGesture>
#include <QPinchGesture>
/**
 * @brief The PaintWidget class
 * 传入相对坐标系，绘制到界面
 */
class PaintWidget: public QWidget{
    Q_OBJECT
public:
    PaintWidget(QWidget* parent=0);
    void zoomIn();  // 放大
    void zoomOut();  // 缩小
    void zoom(qreal scale); // 缩放 - scaleFactor：缩放的比例因子
    void translate(QPointF delta);  // 平移
    void acceptQPoint(QPointF point);
    void addQPoint(QPointF point);
    void clear();
    void initScreenCenter(QPointF);
    ~PaintWidget();
private:
    QList<QPointF> routeLocationList;//存的XY
    QList<QPointF> passwayLocationList;//存的XY
    QPointF centerGPS;

//    QPointF center;//高斯坐标表示
//    QPointF centerScreen;//屏幕坐标表示
    qreal horizontalOffset;
    qreal verticalOffset;
    qint16 m_zoomDelta;  // 缩放值的位置,DELTA[m_zoomDelta]
    qreal scaleFactor;
    qreal currentStepScaleFactor;   //当前比例
    Qt::MouseButton m_translateButton;  // 平移按钮
    bool m_bMouseTranslate;

    QPoint m_lastMousePos;  // 鼠标最后按下的位置

    CCoordinate coordinate;

    QPointF startPoint;
    QPointF nextTargetPoint;

    QLabel* mousePosInfoLabel;
    QLabel* scaleInfoLabel;
    //qreal DELTA[PAINTWIDGET_DELTA_MAX+1]={0.05,0.1,0.2,0.5,1,2,5,10,20,50,100,200,500,1000,2000,5000,10000,50000,100000};
    qreal DELTA[PAINTWIDGET_DELTA_MAX+1]={0.05,0.1,0.2,0.5,1,2,5,7,10,20,100,200,500,1000,2000,5000,10000,50000,100000};


protected:
    void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;
    void mouseDoubleClickEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
    // 平移
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
    void resizeEvent(QResizeEvent *e) Q_DECL_OVERRIDE;
public:
    void paintStartPoint(QPointF GpsInfo);
    void paintTargetPoint(int target);

};
#endif // PAINTWIDGET_H
