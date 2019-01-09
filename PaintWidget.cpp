#include"PaintWidget.h"
#include<QPainter>
#include<QPen>]
#include<QDebug>
PaintWidget::PaintWidget(QWidget *parent):QWidget(parent){
    resize(800, 600);
    setWindowTitle(tr("GPS坐标系统"));
}

PaintWidget::~PaintWidget(){

}

void PaintWidget::paintEvent(QPaintEvent *){
    painter=new QPainter(this);
    painter->drawLine(80, 100, 650, 500);
    painter->setPen(Qt::red);
    painter->drawRect(10, 10, 100, 400);
    painter->setPen(QPen(Qt::green, 5));
    painter->setBrush(Qt::blue);
    painter->drawEllipse(50, 150, 400, 200);
}

void PaintWidget::wheelEvent(QWheelEvent * event){
    qDebug() << "CProjectionPicture::wheelEvent";
}
