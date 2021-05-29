#include "MainWindow.h"
#include <QApplication>
#include <QDebug>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qDebug()<< "Hello word \n";
    MainWindow w;
    w.show();
    return a.exec();
}
