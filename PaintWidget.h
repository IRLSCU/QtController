#ifndef PAINTWIDGET_H
#define PAINTWIDGET_H
#include"CoTrans.h"
#include <QPoint>
#include<QLabel>
#include <QWidget>
#include <QList>
#include <QGestureEvent>
#include <QPanGesture>
#include <QPinchGesture>
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
    ~PaintWidget();
private:
    QList<QPointF> pointList;
    QList<QPointF> passWay;
    QPointF currenGPS;
//    QPointF center;//高斯坐标表示
//    QPointF centerScreen;//屏幕坐标表示
    qreal horizontalOffset;
    qreal verticalOffset;

    qreal scaleFactor;
    qreal currentStepScaleFactor;   //当前比例
    Qt::MouseButton m_translateButton;  // 平移按钮
    bool m_bMouseTranslate;
    qreal m_zoomDelta;  // 缩放的增量
    QPoint m_lastMousePos;  // 鼠标最后按下的位置

    CCoordinate coordinate;
    QLabel* mousePosInfoLabel;
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

//    QPointF XY2Screen(QPointF);
//    QPointF Screen2XY(QPointF);
};
#endif // PAINTWIDGET_H
