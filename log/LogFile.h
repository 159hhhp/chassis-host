#pragma once
/// @file LogFile.h
/// @brief 带滚动的日志文件：按大小 / 跨天滚动，周期性 fflush
///        文件名：basename.YYYYmmdd-HHMMSS.hostname.pid.log

#include <memory>
#include <mutex>
#include <string>

namespace mhost {

namespace detail {
class AppendFile;
}

class LogFile {
 public:
  /// @param threadSafe 仅后台单线程写时可传 false，免锁
  LogFile(const std::string& basename, size_t rollSize, bool threadSafe = true,
          int flushInterval = 3, int checkEveryN = 1024);
  ~LogFile();

  void append(const char* logline, int len);
  void flush();

 private:
  void append_unlocked(const char* logline, int len);
  void rollFile();
  static std::string getLogFileName(const std::string& basename, time_t* now);

  static const int kRollPerSeconds_ = 24 * 60 * 60;  ///< 滚动检查的时间粒度（1 天）

  const std::string basename_;
  const size_t rollSize_;
  const int flushInterval_;
  const int checkEveryN_;

  int count_;  ///< 跨天/定期 flush 检查的节流计数
  std::unique_ptr<std::mutex> mutex_;
  std::unique_ptr<detail::AppendFile> file_;
  time_t startOfPeriod_;  ///< 当前文件所属“天”的起点（秒）
  time_t lastRoll_;
  time_t lastFlush_;
};

}  // namespace mhost
