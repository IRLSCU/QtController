#include "MainWindow.h"
#include "PaintWidget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    w.setCentralWidget(new PaintWidget);
    return a.exec();
}
