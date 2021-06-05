/**
 * @file RingBuffer.h
 * @brief 线程安全环形缓冲区读写类头文件
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2021-06-05 18:17:03
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
 *    <td> 2021-06-05 18:17:03 </td>
 *    <td> 1.0 </td>
 *    <td> wangpengcheng </td>
 *    <td> 添加文档注释 </td>
 * </tr>
 * </table>
 */
#ifndef RING_BUFFER
#define RING_BUFFER
#include <boost/atomic.hpp>

#include "GpsInfo.h"
#include "LocationPosition.h"

/**
 * @brief  基础数据类型类模板
 * @tparam T 中间数据数组长度
 * @tparam Size 大小
 */
template <typename T, size_t Size>
class RingBuffer
{
public:
    /**
     * @brief Construct a new RingBuffer object
     */
    RingBuffer() : head_(0), tail_(0) {}
    /**
     * @brief  添加数据
     * @param  value            数据值
     * @return true             数据放入成功
     * @return false            数据添加失败
     */
    bool push(const T &value)
    {
        size_t head = head_.load(boost::memory_order_relaxed);
        size_t next_head = next(head);
        if (next_head == tail_.load(boost::memory_order_acquire))
            return false;
        ring_[head] = value;
        head_.store(next_head, boost::memory_order_release);
        return true;
    }
    /**
     * @brief  取出数据
     * @param  value            数据值
     * @return true             数据获取成功
     * @return false            数据获取失败
     */
    bool pop(T &value)
    {
        size_t tail = tail_.load(boost::memory_order_relaxed);
        if (tail == head_.load(boost::memory_order_acquire))
            return false;
        value = ring_[tail];
        tail_.store(next(tail), boost::memory_order_release);
        return true;
    }

private:
    /**
     * @brief 缓冲区头部指针向前移动
     * @param  current          当前索引
     * @return size_t           映射后索引
     */
    size_t next(size_t current)
    {
        return (current + 1) % Size;
    }

    T ring_[Size];               ///< 缓冲区数组
    boost::atomic<size_t> head_; ///< 缓冲区头部指针
    boost::atomic<size_t> tail_; ///< 缓冲区尾部指针
};
/**
 * @brief 字符环形缓冲区
 * @details 基础数据类型为char长度为20480
 */
typedef RingBuffer<char, 20480> CharRingBuffer;
/**
 * @brief GPS信息环形缓冲区
 * @details 基础数据类型为GpsInfo  大小为2048
 */
typedef RingBuffer<GpsInfo, 2048> GpsRingBuffer;
/**
 * @brief 相对定位缓冲区
 * @details 基础数据类型为LocationPosition， 长度为2048
 */
typedef RingBuffer<LocationPosition, 2048> LocationRingBuffer;
#endif
