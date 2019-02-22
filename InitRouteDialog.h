#ifndef INITROUTEDIALOG_H
#define INITROUTEDIALOG_H

#include "gpsinfo.h"
#include <QDialog>

namespace Ui {
class InitRouteDialog;
}

class InitRouteDialog : public QDialog
{
    Q_OBJECT

public:
    explicit InitRouteDialog(QWidget *parent = 0);
    void acceptGpsInfo(GpsInfo& GpsInfo);
    void updateBroswerText(GpsInfo GpsInfo);
    void startInit();
    void continueInit();
    void endInit();
    void saveFile();
    ~InitRouteDialog();

private:
    Ui::InitRouteDialog *ui;

signals:
    void sendInitSignal(bool signal);
};

#endif // INITROUTEDIALOG_H
