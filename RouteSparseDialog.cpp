#include "RouteSparseDialog.h"
#include "ui_RouteSparseDialog.h"
#include "CoTrans.h"
#include <qmath.h>
#include<QMessageBox>
#include<QFileDialog>
RouteSparseDialog::RouteSparseDialog(bool isXYZ,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RouteSparseDialog)
{
    ui->setupUi(this);

    this->isXYZ=true;
    this->sparseStatus=false;
    this->loadStatus=false;
    routeGps=new QList<GpsInfo>;
    routeLocation=new QList<LocationPosition>;
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
    QString s="./../QtControl/";
    routeGps->clear();
    routeLocation->clear();
    if(isXYZ){
        s.append("routeXYZ");
    }
    else {
        s.append("route");
    }
    QString path = QFileDialog::getOpenFileName(this,
                                                tr("Open File"),
                                                s);
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
            QString s=in.readLine();
            if(s.length()<10)
                continue;
            if(isXYZ){
                LocationPosition location=LocationPosition::stringToLocation(s);
                routeLocation->append(location);
            }else{
                GpsInfo gpsInfo=GpsInfo::StringToGpsInfo(s);
                routeGps->append(gpsInfo);
//                qreal lon=0;
//                qreal lat=0;
//                int quality=0;
//                double time=0;
//                unsigned long date=0;
//                qreal altitude=0;
//                qreal speed=0;
//                qreal course=0;
//                //分别为经度、纬度、质量、时分秒、年月日、高度、速度、航向
//                in>>lon>>lat>>quality>>time>>date>>altitude>>speed>>course;
//                if(lon==0&&lat==0)
//                    continue;
//                routeGps->append(GpsInfo(lon,lat,quality,time,date,altitude,speed,course));
            }

        }
        file.close();
        if(isXYZ){
            this->setLoadStatus(!(this->loadStatus),routeLocation->size());
        }else {
            this->setLoadStatus(!(this->loadStatus),routeGps->size());
        }
    } else {
        QMessageBox::warning(this, tr("Path"),
                             tr("You did not select any file."));
    }
}
void RouteSparseDialog::saveAsFile(){
    QString s="./../QtControl/";
    if(isXYZ){
        s.append("routeXYZ");
    }
    else {
        s.append("route");
    }
    QString path = QFileDialog::getSaveFileName(this,
                                                tr("Open File"),
                                                s);
    if(!path.isEmpty()) {
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, tr("Write File"),
                                       tr("Cannot open file:\n%1").arg(path));
            return;
        }
        QTextStream out(&file);
        if(isXYZ){
            QList<LocationPosition>::iterator location;
            for(location=routeLocation->begin();location<routeLocation->end();location++){
                out<<location->toString()<<"\n";
            }
        }else{
            QList<GpsInfo>::iterator gps;
    //        QString lon,lat;
            for(gps=routeGps->begin();gps<routeGps->end();gps++){
    //            out<<lon.setNum(gps->longitude,'f',10)<<" "<<lat.setNum(gps->latitude,'f',10)<<" "<<QString::number(gps->quality)
    //              <<" "<<QString::number(gps->time,'f')<<" "<<QString::number(gps->date)<<" "<<QString::number(gps->altitude,'f',2)
    //              <<" "<<QString::number(gps->speed,'f',2)<<" "<<QString::number(gps->course,'f',2)<<"\n";
                out<<gps->toString()<<"\n";
            }
        }

        file.close();
    }
}
void RouteSparseDialog::sparseRoute(){
    bool isSuccessStart,isSuccessEnd,isSuccess;
    double minD=this->ui->factorStartEdit->text().toDouble(&isSuccessStart);
    double maxD=this->ui->factorEndEdit->text().toDouble(&isSuccessEnd);
    isSuccess=isSuccessStart&&isSuccessEnd;
    if(!isSuccess){
        QMessageBox::warning(this, tr("error"),
                                   tr("please input correct number:\n"));
        return;
    }
    if(isXYZ){
        if(routeLocation->size()>0){
            sparseXYZRouteHelper(minD,maxD);
            qDebug()<<"routeLocation's  num :"<<routeLocation->size()<<". sparse distance "<<minD<<"-"<<maxD<<" metre";
        }else{
            QMessageBox::warning(this, tr("error"),
                                       tr("please choose a correct file:\n"));
            return;
        }
    }else{
        if(routeGps->size()>0){
            sparseRouteHelper(minD,maxD);
            qDebug()<<"routeGps's  num :"<<routeGps->size()<<". sparse distance "<<minD<<"-"<<maxD<<" metre";
        }else{
            QMessageBox::warning(this, tr("error"),
                                       tr("please choose a correct file:\n"));
            return;
        }
    }
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
                 qDebug() << QStringLiteral("第") << count << QStringLiteral("个点与第") << count - 1 << QStringLiteral("个点距离为") << distance << QStringLiteral("米");
            }
        }
    }
    delete routeGps;
    routeGps=tempList;
    setSparseStatus(true,routeGps->size());
}
void RouteSparseDialog::sparseXYZRouteHelper(double minD,double maxD){
    int count=0;
    LocationPosition cur=routeLocation->at(0);
    count++;
    QList<LocationPosition>* tempList=new QList<LocationPosition>;
    tempList->append(cur);

    QPointF point_first(cur.x,cur.y);
    for(int i=1;i<routeLocation->size();i++){
        cur=routeLocation->at(i);
        QPointF point_second(cur.x,cur.y);
        double distance = getDistance(point_first, point_second);
        if (distance >= minD) {
            count++;
            tempList->append(cur);
            point_first = point_second;
            if (distance > maxD) {
                 qDebug() << QStringLiteral("第") << count << QStringLiteral("个点与第") << count - 1 << QStringLiteral("个点距离为") << distance << QStringLiteral("米");
            }
        }
    }
    delete routeLocation;
    routeLocation=tempList;
    setSparseStatus(true,routeLocation->size());
}
void RouteSparseDialog::setSparseStatus(bool status,int num){
    sparseStatus=status;
    QString content;
    if(sparseStatus){
        content=QStringLiteral("已完成,稀疏后有点（个）：");
        content+=QString::number(num);
    }else{
        content=QStringLiteral("未完成");
    }
    this->ui->sparseStatusLabel->setText(content);
}
void RouteSparseDialog::setLoadStatus(bool status,int num){
    loadStatus=status;
    QString content;
    if(loadStatus){
        content=QStringLiteral("已加载,GPS点共有:");
        content+=QString::number(num);
    }else{
        content=QStringLiteral("未加载");
    }
    this->ui->loadStatusLabel->setText(content);
}
double RouteSparseDialog::getDistance(QPointF first, QPointF second) {
    return sqrt((first.x() - second.x())*(first.x() - second.x()) + (first.y() - second.y())*(first.y() - second.y()));
}
