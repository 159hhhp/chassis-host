#pragma once
/// @file AsyncLogging.h
/// @brief 双缓冲异步日志后端
///
/// 前端（各业务线程）append() 只做一次加锁拷贝：缓冲够则直接返回
/// （不唤醒、不分配、不碰磁盘）；写满才换缓冲并唤醒后台。后台线程至多
/// 等 flushInterval_(3s) 或被唤醒，把满缓冲整批写入 LogFile；积压超限
/// 丢弃并告警；写完的缓冲回收复用。具体策略见 AsyncLogging.cc 内注释
/// 与 docs/logging.md。

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "LogStream.h"

namespace mhost {

class LogFile;

/// 双缓冲异步日志器：append() 线程安全且非阻塞（时间 O(拷贝)，不碰磁盘）。
class AsyncLogging {
 public:
  AsyncLogging(std::string basename, size_t rollSize, int flushInterval = 3);
  ~AsyncLogging();

  void append(const char* logline, int len);

  void start();
  void stop();

 private:
  void threadFunc();

  typedef detail::FixedBuffer<detail::kLargeBuffer> Buffer;
  typedef std::vector<std::unique_ptr<Buffer>> BufferVector;
  typedef BufferVector::value_type BufferPtr;

  const int flushInterval_;          ///< 后台落盘周期（秒）
  std::atomic<bool> running_;
  const std::string basename_;       ///< 日志文件名前缀
  const size_t rollSize_;            ///< 单文件滚动阈值（字节）
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cond_;
  BufferPtr currentBuffer_;          ///< 前台当前缓冲
  BufferPtr nextBuffer_;             ///< 前台备用缓冲
  BufferVector buffers_;             ///< 待后台落盘的满缓冲队列
};

}  // namespace mhost
