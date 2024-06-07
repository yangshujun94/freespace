/*
* file: read-write lock : copy from 3rdframework
* author: Yutao
* data: 2022/1/6
*/

#ifndef RWLOCK_H
#define RWLOCK_H

#include <thread>
#include <atomic>
#include <cstdlib>
#include <cassert>
#include "raii.h"

namespace fs
{

  class RWLock
  {
#define WRITE_LOCK_STATUS -1
#define FREE_STATUS       0
  private:
    /* 初始为0的线程id */
    static const std::thread::id NULL_THEAD;
    const bool                   WRITE_FIRST;
    /* 用于判断当前是否是写线程 */
    std::thread::id m_write_thread_id;
    /* 资源锁计数器,类型为int的原子成员变量,-1为写状态，0为自由状态,>0为共享读取状态 */
    std::atomic_int m_lockCount;
    /* 等待写线程计数器,类型为unsigned int的原子成员变量*/
    std::atomic_uint m_writeWaitCount;

  public:
    // 禁止复制构造函数
    RWLock(const RWLock&) = delete;
    // 禁止对象赋值操作符
    RWLock& operator=(const RWLock&) = delete;
    //RWLock& operator=(const RWLock&) volatile = delete;
    RWLock(bool writeFirst = false); //默认为读优先模式
    virtual ~RWLock() = default;
    int readLock();
    int readUnlock();
    int writeLock();
    int writeUnlock();
    // 将读取锁的申请和释放动作封装为raii对象，自动完成加锁和解锁管理
    raii read_guard() const noexcept
    {
      return make_raii(*this, &RWLock::readUnlock, &RWLock::readLock);
    }
    // 将写入锁的申请和释放动作封装为raii对象，自动完成加锁和解锁管理
    raii write_guard() noexcept
    {
      return make_raii(*this, &RWLock::writeUnlock, &RWLock::writeLock);
    }
  };

} // namespace fs

#endif
