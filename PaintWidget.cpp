#include"PaintWidget.h"
#include<QPainter>
#include<QPen>
#include<QDebug>

PaintWidget::PaintWidget(QWidget *parent)
    : QWidget(parent),
    horizontalOffset(0),
    verticalOffset(0),
    scaleFactor(1),
    currentStepScaleFactor(1),
    m_translateButton(Qt::LeftButton),
    m_bMouseTranslate(false),
    m_zoomDelta(0.2)
{
    this->setFocusPolicy(Qt::ClickFocus);

    grabGesture(Qt::PanGesture);
    grabGesture(Qt::PinchGesture);
    grabGesture(Qt::SwipeGesture);
}
PaintWidget::~PaintWidget(){

}

void PaintWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    const qreal wh = height();
    const qreal ww = width();

    painter.translate(ww/2, wh/2);
    painter.translate(horizontalOffset, verticalOffset);
    painter.scale(currentStepScaleFactor * scaleFactor, currentStepScaleFactor * scaleFactor);

    painter.drawRect(0,0,100,100);
}
//双击初始化
void PaintWidget::mouseDoubleClickEvent(QMouseEvent *)
{
    scaleFactor = 1;
    currentStepScaleFactor = 1;
    verticalOffset = 0;
    horizontalOffset = 0;
    update();
}

void PaintWidget::resizeEvent(QResizeEvent*e)
{
    update();
    QWidget::resizeEvent(e);
}

// 上/下/左/右键向各个方向移动、加/减键进行缩放
void PaintWidget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    qDebug() << event->key();
    case Qt::Key_Up:
        translate(QPointF(0, -5));  // 上移
        break;
    case Qt::Key_Down:
        translate(QPointF(0, 5));  // 下移
        break;
    case Qt::Key_Left:
        translate(QPointF(-5, 0));  // 左移
        break;
    case Qt::Key_Right:
        translate(QPointF(5, 0));  // 右移
        break;
    case Qt::Key_Plus:  // 放大
        zoomIn();
        break;
    case Qt::Key_Minus:  // 缩小
        zoomOut();
        break;
    default:
        QWidget::keyPressEvent(event);
    }
    QWidget::keyPressEvent(event);
}

// 平移,通过一个标志位判断是否按下鼠标--mousePressEvent&mouseReleaseEvent
void PaintWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_bMouseTranslate){
        QPointF mouseDelta = event->pos() - m_lastMousePos;
        translate(mouseDelta);
    }

    m_lastMousePos = event->pos();

    QWidget::mouseMoveEvent(event);
}

void PaintWidget::mousePressEvent(QMouseEvent *event)
{
    qDebug() << "PaintWidget::mousePressEvent";
    if (event->button() == m_translateButton) {
        m_bMouseTranslate = true;
        m_lastMousePos = event->pos();
        setCursor(Qt::OpenHandCursor);
    }
    QWidget::mousePressEvent(event);
}

void PaintWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == m_translateButton)
    {
        m_bMouseTranslate = false;
        setCursor(Qt::ArrowCursor);
    }

    QWidget::mouseReleaseEvent(event);
}

// 放大/缩小
void PaintWidget::wheelEvent(QWheelEvent *event)
{
    qDebug() << "PaintWidget::wheelEvent";
    QPoint scrallAmount = event->angleDelta();
    if(scrallAmount.y() > 0){
        zoomIn();
    }
    else if(scrallAmount.y() < 0){
        zoomOut();
    }
    QWidget::wheelEvent(event);
}

// 放大
void PaintWidget::zoomIn()
{
    zoom(1 + m_zoomDelta);
}

// 缩小
void PaintWidget::zoomOut()
{
    zoom(1 - m_zoomDelta);
}

// 缩放 - scaleFactor：缩放的比例因子
void PaintWidget::zoom(qreal scale)
{
    scaleFactor *= scale;
    update();
}

// 平移
void PaintWidget::translate(QPointF delta)
{
    horizontalOffset += delta.x();
    verticalOffset += delta.y();
    update();
}

void PaintWidget::acceptQPoint(QPoint point){
    pointList.push_back(point);
    //qDebug()<<pointList.at(0).x();
}
