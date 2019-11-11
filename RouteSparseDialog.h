#ifndef ROUTESPARSEDIALOG_H
#define ROUTESPARSEDIALOG_H
#include "GpsInfo.h"
#include "LocationPosition.h"
#include <QDialog>
#include<QList>
#include<QPointF>
namespace Ui {
class RouteSparseDialog;
}
/**
 * @brief The RouteSparseDialog class
 * 稀疏路径
 * 输入：文件
 * 输出：文件
 * 处理：将gps点按照输入的距离稀疏
 */
class RouteSparseDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RouteSparseDialog(bool isXYZ=false,QWidget *parent = 0);
    void init();
    void openFile();
    void saveAsFile();
    void sparseRoute();
    void sparseRouteHelper(double minDistance,double maxDistance);
    void sparseXYZRouteHelper(double minDistance,double maxDistance);
    void setSparseStatus(bool,int);
    void setLoadStatus(bool,int);
    ~RouteSparseDialog();

private:
    Ui::RouteSparseDialog *ui;
    QList<GpsInfo>* routeGps;
    bool isXYZ;
    QList<LocationPosition>* routeLocation;
    bool sparseStatus;
    bool loadStatus;
    double getDistance(QPointF,QPointF);
};

#endif // ROUTESPARSEDIALOG_H
