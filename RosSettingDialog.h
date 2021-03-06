#ifndef ROSSETTINGDIALOG_H
#define ROSSETTINGDIALOG_H

#include "LocationBufferProduceThread.h"
#include "RosReceiveThread.h"
#include <QDialog>

namespace Ui {
class RosSettingDialog;
}

class RosSettingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RosSettingDialog(LocationRingBuffer*,QWidget *parent = nullptr);
    ~RosSettingDialog();
    void clear();
    void save();
    void updateTextBrowser(QString);
private:
    Ui::RosSettingDialog *ui;
    RosReceiveThread* rosReceiveThread;
};

#endif // ROSSETTINGDIALOG_H
