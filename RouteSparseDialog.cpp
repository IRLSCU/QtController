#include "RouteSparseDialog.h"
#include "ui_RouteSparseDialog.h"
#include "CoTrans.h"
#include<QMessageBox>
#include<QFileDialog>
RouteSparseDialog::RouteSparseDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RouteSparseDialog)
{
    ui->setupUi(this);
    this->sparseStatus=false;
    this->loadStatus=false;
    routeGps=new QList<GpsInfo>;

    connect(ui->openFileButton,&QPushButton::clicked,this,&RouteSparseDialog::openFile);
    connect(ui->sparseButton,&QPushButton::clicked,this,&RouteSparseDialog::sparseRoute);
    connect(ui->saveAsButton,&QPushButton::clicked,this,&RouteSparseDialog::saveAsFile);
    connect(ui->initButton,&QPushButton::clicked,this,&RouteSparseDialog::init);

}
void RouteSparseDialog::init(){
    routeGps->clear();
    setSparseStatus(false,0);
    setLoadStatus(false,0);
}
RouteSparseDialog::~RouteSparseDialog()
{
    delete routeGps;
    delete ui;
}
void RouteSparseDialog::openFile(){
    QString path = QFileDialog::getOpenFileName(this,
                                                tr("Open File"),
                                                "./../QtControl/route",
                                                tr("Text Files(*.txt)"));
    if(!path.isEmpty()) {
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QMessageBox::warning(this, tr("Read File"),
                                 tr("Cannot open file:\n%1").arg(path));
            return;
        }
        init();
        QTextStream in(&file);
        while (!in.atEnd()) {
            qreal x=0;
            qreal y=0;
            in>>x>>y;
            if(x==0&&y==0)
                continue;
            routeGps->append(GpsInfo(x,y));
        }
        file.close();
        this->setLoadStatus(!(this->loadStatus),routeGps->size());
    } else {
        QMessageBox::warning(this, tr("Path"),
                             tr("You did not select any file."));
    }
}
void RouteSparseDialog::saveAsFile(){
    QString path = QFileDialog::getSaveFileName(this,
                                                tr("Open File"),
                                                "./../QtControl/route",
                                                tr("Text Files(*.txt)"));
    if(!path.isEmpty()) {
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, tr("Write File"),
                                       tr("Cannot open file:\n%1").arg(path));
            return;
        }
        QTextStream out(&file);
        QList<GpsInfo>::iterator gps;
        QString lon,lat;
        for(gps=routeGps->begin();gps<routeGps->end();gps++){
            out<<lon.setNum(gps->longitude,'f',10)<<" "<<lat.setNum(gps->latitude,'f',10)<<"\n";
        }
        file.close();
    }
}
void RouteSparseDialog::sparseRoute(){
    bool isSuccessStart,isSuccessEnd,isSuccess;
    double minD=this->ui->factorStartEdit->text().toDouble(&isSuccessStart);
    double maxD=this->ui->factorEndEdit->text().toDouble(&isSuccessEnd);
    isSuccess=isSuccessStart&&isSuccessEnd;
    if(isSuccess&&routeGps->size()>0){
        sparseRouteHelper(minD,maxD);
    }else{
        if(routeGps->size()<=0){
            QMessageBox::warning(this, tr("error"),
                                       tr("please choose a correct file:\n"));
            return;
        }
        if(!isSuccess){
            QMessageBox::warning(this, tr("error"),
                                       tr("please input correct number:\n"));
            return;
        }
    }
    qDebug()<<"routeGps's  num :"<<routeGps->size()<<". sparse distance "<<minD<<"-"<<maxD<<" metre";
}

void RouteSparseDialog::sparseRouteHelper(double minD,double maxD){
    int count=0;
    CCoordinate coordinate;
    GpsInfo cur=routeGps->at(0);
    count++;
    coordinate.InitRadarPara(500, cur.longitude,cur.latitude);//初始化坐标原点

    QList<GpsInfo>* tempList=new QList<GpsInfo>;
    tempList->append(cur);

    LongLat lonlat_first, lonlat_second;
    lonlat_first.Lon=cur.longitude;
    lonlat_first.Lat=cur.latitude;
    QPointF point_first = coordinate.LongLat2XY(lonlat_first);
    for(int i=1;i<routeGps->size();i++){
        cur=routeGps->at(i);
        lonlat_second.Lon=cur.longitude;
        lonlat_second.Lat=cur.latitude;
        QPointF point_second = coordinate.LongLat2XY(lonlat_second);
        double distance = getDistance(point_first, point_second);
        if (distance >= minD) {
            count++;
            tempList->append(cur);
            lonlat_first = lonlat_second;
            point_first = point_second;
            if (distance > maxD) {
                 qDebug() << "第" << count << "个点与第" << count - 1 << "个点距离为" << distance << "米\n";
            }
        }
    }
    delete routeGps;
    routeGps=tempList;
    setSparseStatus(true,routeGps->size());
}
void RouteSparseDialog::setSparseStatus(bool status,int num){
    sparseStatus=status;
    QString content;
    if(sparseStatus){
        content.sprintf("已完成,稀疏后有%d个点",num);
    }else{
        content="未完成";
    }
    this->ui->sparseStatusLabel->setText(content);
}
void RouteSparseDialog::setLoadStatus(bool status,int num){
    loadStatus=status;
    QString content;
    if(loadStatus){
        content.sprintf("已加载,共%d个点",num);
    }else{
        content="未加载";
    }
    this->ui->loadStatusLabel->setText(content);
}
double RouteSparseDialog::getDistance(QPointF first, QPointF second) {
    return sqrt((first.x() - second.x())*(first.x() - second.x()) + (first.y() - second.y())*(first.y() - second.y()));
}
