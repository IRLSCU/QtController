#include"PaintWidget.h"
#include<QHBoxLayout>
#include<QPushButton>
#include<QVBoxLayout>
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
    this->resize(600,400);
    coordinate=CCoordinate();
    coordinate.InitRadarPara(500, 103.9588080550,30.7852871117);
    mousePosInfoLabel=new QLabel("");
    mousePosInfoLabel->setParent(this);
    QPushButton* addPointButton=new QPushButton("add point");
    connect(addPointButton,&QPushButton::clicked,[this](){
        //addQPoint(QPointF(qrand()%width()-width()/2,qrand()%height()-height()/2));
        addQPoint(coordinate.LongLat2Screen(LongLat(103.9588080550,30.7852871117)));
    });
    QHBoxLayout *layout = new QHBoxLayout;
    QVBoxLayout *layout2 = new QVBoxLayout;
    layout2->addStretch();
    layout->addStretch();
    layout->addLayout(layout2);
    layout2->addWidget(addPointButton);
    layout2->addWidget(mousePosInfoLabel);
    this->setLayout(layout);


}
PaintWidget::~PaintWidget(){

}

void PaintWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const qreal wh = height();
    const qreal ww = width();
    painter.translate(ww/2, wh/2);
    painter.translate(horizontalOffset, verticalOffset);
    coordinate.m_DspCenter=QPointF(0,0);
    painter.scale(currentStepScaleFactor * scaleFactor, currentStepScaleFactor * scaleFactor);
    QPen mypen;
    mypen.setWidth(0);                     // 1 表示点的大小（形状为方形）
    mypen.setColor(Qt::black);
    mypen.setCapStyle(Qt::RoundCap);
    painter.setPen(mypen);
    for(auto it:pointList){
        QPointF tmep=coordinate.XY2Screen(it);
        painter.drawPoint((qint64)tmep.x(),(qint64)tmep.y());
    }
    mypen.setColor(Qt::green);
    painter.setPen(mypen);
    for(auto it:passWay){
        QPointF tmep=coordinate.XY2Screen(it);
        painter.drawPoint((qint64)tmep.x(),(qint64)tmep.y());
    }

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

    LongLat mouseGPS=coordinate.Screen2LongLat((event->pos().x()-width()/2-horizontalOffset)/currentStepScaleFactor/scaleFactor,(event->pos().y()-height()/2-verticalOffset)/currentStepScaleFactor/scaleFactor);
    qDebug()<<"偏移："<<horizontalOffset<<verticalOffset;
    QString lat=tr("lat:")+QString::number(mouseGPS.Lat,10,10);
    QString lon=tr(" lon:")+QString::number(mouseGPS.Lon,10,10);
    mousePosInfoLabel->setText(lat+lon);
    mousePosInfoLabel->adjustSize();
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

void PaintWidget::acceptQPoint(QPointF point){
    QPointF screenPoint=coordinate.LongLat2XY(LongLat(point.x(),point.y()));
    pointList.push_back(screenPoint);
}
void PaintWidget::addQPoint(QPointF point){
    passWay.push_back(point);
    update();
}
