/**
 * @file fileoperation.h
 * @brief FileOperation文件读写抽象类头文件
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-04 15:47:56
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
 *    <td> 2021-06-04 15:47:56 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加文档注释 </td>
 * </tr>
 * </table>
 */
#ifndef FILEOPERATION_H
#define FILEOPERATION_H
#include <QString>
#include <QFile>

/**
 * @brief 文件操作抽象类
 * @details 主要用于对文件操作进行抽象和进一步操作
 */
class FileOperation
{
public:
    /**
     * @brief Construct a new File Operation object
     * @details 空构造函数无实际意义
     */
    FileOperation();
    /**
     * @brief  文件读写static操作
     * @param  filePath         文件全局路径
     * @return QString          文件读取结果
     */
    static QString readFile(QString filePath);
    QString filePasth; ///< 操作对象文件路径
};

#endif // FILEOPERATION_H
