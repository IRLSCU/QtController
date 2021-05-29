/**
 * @file main.cpp
 * @brief  项目主要程序入口，主要包含main执行函数类
 * @todo 与GUI进行分离，尝试使用剥离Qt依赖
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-05-29 22:06:41
 * @copyright Copyright (c) 2021  IRLSCU
 * 
 * @par 修改日志:
 * <table>
 * <tr>
 *    <th> Commit date</th>
 *    <th> Version </th> 
 *    <th> Author </th>  
 *    <th> Description </th>
 * </tr>
 * <tr>
 *    <td> 2021-05-29 22:06:41 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加代码文档 </td>
 * </tr>
 * </table>
 */
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
