#ifndef PAINTWIDGET_H
#define PAINTWIDGET_H

#include <QPoint>
#include <QWidget>
#include <QList>
class PaintWidget: public QWidget{
    Q_OBJECT
public:
    PaintWidget(QWidget* parent=0);
    ~PaintWidget();
private:
    QList<QPoint> pointList;
    QPainter* painter;
protected:
    void paintEvent(QPaintEvent *);
    void wheelEvent(QWheelEvent * event);
};
#endif // PAINTWIDGET_H
