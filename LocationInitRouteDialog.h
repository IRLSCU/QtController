#ifndef LOCATIONINITROUTEDIALOG_H
#define LOCATIONINITROUTEDIALOG_H

#include "LocationBufferConsumInitRouteThread.h"
#include "RingBuffer.h"
#include <QDialog>

namespace Ui {
class LocationInitRouteDialog;
}

class LocationInitRouteDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LocationInitRouteDialog(LocationRingBuffer*,QWidget *parent = nullptr);
    void acceptGpsInfo(LocationPosition& location);
    void updateBroswerText(QString location);
    void startInit();
    void continueInit();
    void endInit();
    void saveFile();
    LocationBufferConsumInitRouteThread* getLocationBufferConsumInitRouteThread(){
        return locationConsumThread;
    }
    ~LocationInitRouteDialog();

private:
    Ui::LocationInitRouteDialog *ui;
    LocationBufferConsumInitRouteThread * locationConsumThread;
    LocationRingBuffer* locationRingBuffer;
signals:
    void sendInitSignal(bool signal);
};

#endif // LOCATIONINITROUTEDIALOG_H
