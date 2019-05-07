#include "RosSettingDialog.h"
#include "ui_RosSettingDialog.h"

#include <QFileDialog>
#include <QMessageBox>
RosSettingDialog::RosSettingDialog(LocationRingBuffer* locationRingBuffer,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RosSettingDialog)
{
    ui->setupUi(this);

    rosReceiveThread=new RosReceiveThread(locationRingBuffer,this);
    connect(rosReceiveThread,&RosReceiveThread::sendMessage,this,&RosSettingDialog::updateTextBrowser);
    rosReceiveThread->start();

    connect(this->ui->clearBT,&QPushButton::clicked,this,&RosSettingDialog::clear);
    connect(this->ui->saveBT,&QPushButton::clicked,this,&RosSettingDialog::save);
}
void RosSettingDialog::updateTextBrowser(QString str){
    ui->textBrowser->insertPlainText(str);
    ui->textBrowser->moveCursor(QTextCursor::End);
}
void RosSettingDialog::clear(){
    ui->textBrowser->clear();
}
void RosSettingDialog::save(){
    QString path = QFileDialog::getSaveFileName(this,
                                                tr("Open File"),
                                                "./../QtControl/rosData",
                                                tr("Text Files(*.txt)"));
    if(!path.isEmpty()) {
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, tr("Write File"),
                                       tr("Cannot open file:\n%1").arg(path));
            return;
        }
        QTextStream out(&file);
        out << ui->textBrowser->toPlainText();
        file.close();
    }
}
RosSettingDialog::~RosSettingDialog()
{
    rosReceiveThread->stopImmediately();
    rosReceiveThread->wait();
    delete ui;
}
