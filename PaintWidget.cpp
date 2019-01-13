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
    QPointF temp=coordinate.LongLat2XY(103.9588080550,30.7852871117);
    coordinate.m_DspCenter.setX(temp.x());
    coordinate.m_DspCenter.setY(temp.y());
//    center=coordinate.LongLat2XY(103.9588080550,30.7852871117);
//    centerScreen=QPointF(0,0);
    mousePosInfoLabel=new QLabel("");
    mousePosInfoLabel->setParent(this);
    QPushButton* addPointButton=new QPushButton("add point");
    connect(addPointButton,&QPushButton::clicked,[this](){
        //addQPoint(QPointF(qrand()%width()-width()/2,qrand()%height()-height()/2));
    });
    QPushButton* clearPointButton=new QPushButton("clear all");
    connect(clearPointButton,&QPushButton::clicked,this,&PaintWidget::clear);
    QHBoxLayout *layout = new QHBoxLayout;
    QVBoxLayout *layout2 = new QVBoxLayout;
    layout2->addStretch();
    layout->addStretch();
    layout->addLayout(layout2);
    layout2->addWidget(clearPointButton);
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
    painter.translate(ww/2, wh/2);//将绘图原点移至中心
    painter.translate(horizontalOffset, verticalOffset);//鼠标拖动的偏移量
    painter.scale(currentStepScaleFactor * scaleFactor, currentStepScaleFactor * scaleFactor);//放大缩小
    //wh/2+horizontalOffset,ww/2+verticalOffset
    coordinate.m_fixedScale=currentStepScaleFactor;
    coordinate.m_scale=scaleFactor;
    qDebug()<<"屏幕像素"<<ww<<wh<<"重绘偏移"<<horizontalOffset<<verticalOffset
           <<"放大倍数"<<currentStepScaleFactor * scaleFactor;

    QPen mypen;
    mypen.setWidth(1);                     // 1 表示点的大小
    mypen.setColor(Qt::black);
    mypen.setCapStyle(Qt::RoundCap);
    painter.setPen(mypen);
    QPointF tmep1=coordinate.LongLat2Screen(LongLat(103.9588080550,30.7852871117));
    painter.drawPoint((qint64)tmep1.x(),(qint64)tmep1.y());
    QPointF tmep2=coordinate.LongLat2Screen(LongLat(103.9587750583,30.7852852883));
    painter.drawPoint((qint64)tmep2.x(),(qint64)tmep2.y());

//    QPointF last;
//    for(int i=0;i<pointList.size();i++){
//        QPointF temp=XY2Screen(pointList.at(i));
//        painter.drawPoint((qint64)temp.x(),(qint64)temp.y());
//        if(i!=0){
//            mypen.setColor(Qt::green);
//            painter.setPen(mypen);
//            painter.drawLine(last,temp);
//            mypen.setColor(Qt::green);
//            painter.setPen(mypen);
//        }
//        last=temp;
//    }
//    mypen.setColor(Qt::green);
//    painter.setPen(mypen);
//    for(auto it:passWay){
//        QPointF tmep=XY2Screen(it);
//        painter.drawPoint((qint64)tmep.x(),(qint64)tmep.y());
//    }

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
    qDebug("screen偏移:(%d,%d)  ",event->pos().x(),event->pos().y());
    qreal x=event->pos().x()-width()/2-horizontalOffset;
    qreal y=event->pos().y()-height()/2-verticalOffset;
    QPointF temp=coordinate.Screen2XY(x,y);
    qDebug("xy偏移:(%.10f,%.10f)  ",temp.x(),temp.y());
    LongLat mouseGPS=coordinate.XY2LongLat(temp);

    QString lat=tr("lat:")+QString::number(mouseGPS.Lat,10,10);
    QString lon=tr(" lon:")+QString::number(mouseGPS.Lon,10,10);
    mousePosInfoLabel->setText(lat+lon);
    mousePosInfoLabel->adjustSize();

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

void PaintWidget::acceptQPoint(QPointF point){
    QPointF screenPoint=coordinate.LongLat2XY(LongLat(point.x(),point.y()));
    pointList.push_back(screenPoint);
}
void PaintWidget::addQPoint(QPointF point){
    passWay.push_back(point);
    update();
}
void PaintWidget::clear(){
    passWay.clear();
    pointList.clear();
    update();
}
//QPointF PaintWidget::XY2Screen(QPointF XY){
//    qreal dx=XY.x()-center.x();
//    qreal dy=XY.y()-center.y();
//    QPointF screen(dx*currentStepScaleFactor*scaleFactor,-dy*currentStepScaleFactor*scaleFactor);
//    return screen;
//}

//QPointF PaintWidget::Screen2XY(QPointF screen){
//    qreal dx=(screen.x()-centerScreen.x())/currentStepScaleFactor/scaleFactor;
//    qreal dy=-(screen.y()-centerScreen.y())/currentStepScaleFactor/scaleFactor;
//    QPointF XY(center.x()+dx,center.y()+dy);
//    qDebug()<<XY.x()<<XY.y();
//    return XY;
//}
