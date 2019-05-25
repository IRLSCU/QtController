#ifndef FILEOPERATION_H
#define FILEOPERATION_H
#include <QString>
#include <QFile>


class FileOperation
{
public:
    QString filePasth;
    FileOperation();
    static QString readFile(QString filePath);
};

#endif // FILEOPERATION_H
