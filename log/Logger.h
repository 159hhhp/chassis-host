#pragma once
/// @file Logger.h
/// @brief 日志前端：六级流式日志宏，行格式
///        "YYYYMMDD HH:MM:SS.uuuuuu tid LEVEL [errno信息] 用户内容 - 文件:行\n"
///
/// 输出去向由 setOutput() 注入：默认写 stdout；main() 启动 AsyncLogging 后
/// 将输出函数替换为 AsyncLogging::append，前端即"只格式化 + 一次加锁拷贝"，
/// 磁盘 I/O 全部交给后台线程（非阻塞）。

#include <functional>
#include <sys/time.h>

#include "LogStream.h"
#include "Timestamp.h"

namespace mhost {

class Logger {
 public:
  enum LogLevel {
    TRACE,  ///< 逐字节/收发流水（量大，默认关闭）
    DEBUG,  ///< 状态机与策略细节
    INFO,   ///< 关键事件（建链、断链、开关串口）
    WARN,   ///< 可自愈异常（重连、静默、补发）
    ERROR,  ///< 单次操作失败
    FATAL,  ///< 不可恢复，打完后 abort
    NUM_LOG_LEVELS,
  };

  /// 编译期裁剪源文件路径（只保留文件名）
  class SourceFile {
   public:
    template <int N>
    inline SourceFile(const char (&arr)[N]) : data_(arr), size_(N - 1) {
      const char* slash = ::strrchr(data_, '/');
      if (slash != nullptr) {
        data_ = slash + 1;
        size_ = static_cast<int>(data_ + size_ - slash - 1);
      }
      const char* bslash = ::strrchr(data_, '\\');
      if (bslash != nullptr) {
        data_ = bslash + 1;
        size_ = static_cast<int>(::strlen(data_));
      }
    }
    explicit SourceFile(const char* filename) : data_(filename) {
      const char* slash = ::strrchr(filename, '/');
      if (slash != nullptr) data_ = slash + 1;
      const char* bslash = ::strrchr(data_, '\\');
      if (bslash != nullptr) data_ = bslash + 1;
      size_ = static_cast<int>(::strlen(data_));
    }
    const char* data_;
    int size_;
  };

  Logger(SourceFile file, int line);
  Logger(SourceFile file, int line, LogLevel level);
  Logger(SourceFile file, int line, LogLevel level, const char* func);
  /// 带系统 errno 的日志：toAbort=true 时析构后 abort()
  Logger(SourceFile file, int line, bool toAbort);

  ~Logger();

  LogStream& stream() { return impl_.stream_; }

  typedef void (*OutputFunc)(const char* msg, int len);
  typedef void (*FlushFunc)();

  static LogLevel logLevel();
  static void setLogLevel(LogLevel level);
  /// CLI 解析用：接受 "trace/debug/info/warn/error/fatal"，非法返回 NUM_LOG_LEVELS
  static LogLevel levelFromString(const char* name);
  static const char* levelName(LogLevel level);

  static void setOutput(OutputFunc outputFunc) { g_output = outputFunc; }
  static void setFlush(FlushFunc flushFunc) { g_flush = flushFunc; }

 private:
  class Impl {
   public:
    Impl(LogLevel level, int savedErrno, const SourceFile& file, int line);
    void formatTime();  ///< 秒级字符串线程内缓存，跨秒才重新 localtime_r
    void finish();

    Timestamp time_;
    LogStream stream_;
    LogLevel level_;
    int line_;
    SourceFile basename_;
  };

  Impl impl_;

  static LogLevel g_logLevel;
  static OutputFunc g_output;
  static FlushFunc g_flush;
};

extern const char* const kLogLevelNames[Logger::NUM_LOG_LEVELS];

inline Logger::LogLevel Logger::logLevel() { return g_logLevel; }
inline void Logger::setLogLevel(LogLevel level) { g_logLevel = level; }

// 六级流式日志宏：低于当前级别直接短路，无格式化开销
#define LOG_TRACE                                         \
  if (mhost::Logger::logLevel() <= mhost::Logger::TRACE)  \
  mhost::Logger(__FILE__, __LINE__, mhost::Logger::TRACE, __func__).stream()
#define LOG_DEBUG                                         \
  if (mhost::Logger::logLevel() <= mhost::Logger::DEBUG)  \
  mhost::Logger(__FILE__, __LINE__, mhost::Logger::DEBUG, __func__).stream()
#define LOG_INFO                                         \
  if (mhost::Logger::logLevel() <= mhost::Logger::INFO)  \
  mhost::Logger(__FILE__, __LINE__, mhost::Logger::INFO).stream()
#define LOG_WARN                                         \
  if (mhost::Logger::logLevel() <= mhost::Logger::WARN)  \
  mhost::Logger(__FILE__, __LINE__, mhost::Logger::WARN).stream()
#define LOG_ERROR                                         \
  if (mhost::Logger::logLevel() <= mhost::Logger::ERROR)  \
  mhost::Logger(__FILE__, __LINE__, mhost::Logger::ERROR).stream()
#define LOG_FATAL                                                \
  if (mhost::Logger::logLevel() <= mhost::Logger::FATAL)         \
  mhost::Logger(__FILE__, __LINE__, mhost::Logger::FATAL).stream()

/// 记录当前 errno 的系统错误（不受级别开关限制）
#define LOG_SYSERR mhost::Logger(__FILE__, __LINE__, false).stream()
#define LOG_SYSFATAL mhost::Logger(__FILE__, __LINE__, true).stream()

}  // namespace mhost
