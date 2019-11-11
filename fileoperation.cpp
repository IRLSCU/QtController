#include "fileoperation.h"

FileOperation::FileOperation()
{

}


QString FileOperation::readFile(QString filePath){
    QFile file(filePath);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QByteArray t = file.readLine();
    file.close();
    return QString(t);
}
