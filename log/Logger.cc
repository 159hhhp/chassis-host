#include "Logger.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <strings.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "CurrentThread.h"

namespace mhost {

namespace detail {
// 默认输出：同步写 stdout（AsyncLogging 未接管时）
void defaultOutput(const char* msg, int len);
void defaultFlush();
}  // namespace detail

const char* const kLogLevelNames[Logger::NUM_LOG_LEVELS] = {
    "TRACE ", "DEBUG ", "INFO  ", "WARN  ", "ERROR ", "FATAL ",
};

Logger::LogLevel Logger::g_logLevel = Logger::INFO;
Logger::OutputFunc Logger::g_output = detail::defaultOutput;
Logger::FlushFunc Logger::g_flush = detail::defaultFlush;

namespace detail {
namespace {
// strerror_r 的线程安全封装（GNU/XSI 两种变体统一处理）
const char* strerror_tl(int savedErrno) {
  thread_local char buf[64];
  return ::strerror_r(savedErrno, buf, sizeof buf);
}
}  // namespace
}  // namespace detail

namespace CurrentThread {
namespace {
thread_local int t_tid = 0;
}  // namespace

void cacheTid() {
  if (t_tid == 0) {
    t_tid = static_cast<int>(::syscall(SYS_gettid));
  }
}

int tid() {
  if (t_tid == 0) cacheTid();
  return t_tid;
}

bool tidCached() { return t_tid != 0; }

}  // namespace CurrentThread

// ---- Logger::Impl ----

Logger::Impl::Impl(LogLevel level, int savedErrno, const SourceFile& file, int line)
    : time_(Timestamp::now()), stream_(), level_(level), line_(line), basename_(file) {
  formatTime();
  // 线程 id（5 位宽）与级别（6 字符定宽，含尾空格）
  stream_ << CurrentThread::tid() << ' ' << kLogLevelNames[level_];
  if (savedErrno != 0) {
    stream_ << detail::strerror_tl(savedErrno) << " (errno=" << savedErrno << ") ";
  }
}

/// 秒级时间字符串缓存在线程局部变量里，同一秒内的日志不做时区换算
void Logger::Impl::formatTime() {
  int64_t microSecondsSinceEpoch = time_.microSecondsSinceEpoch();
  time_t seconds = static_cast<time_t>(microSecondsSinceEpoch / Timestamp::kMicroPerSecond);
  thread_local char t_time[64];
  thread_local time_t t_lastSecond = 0;
  if (seconds != t_lastSecond) {
    t_lastSecond = seconds;
    struct tm tm_time;
    ::localtime_r(&seconds, &tm_time);
    ::snprintf(t_time, sizeof t_time, "%4d%02d%02d %02d:%02d:%02d",
               tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday,
               tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);
  }
  Fmt us(".%06d ", static_cast<int>(microSecondsSinceEpoch % Timestamp::kMicroPerSecond));
  stream_ << t_time << us;
}

void Logger::Impl::finish() {
  stream_ << " - " << basename_.data_ << ':' << line_ << '\n';
}

// ---- Logger ----

Logger::Logger(SourceFile file, int line) : impl_(INFO, 0, file, line) {}
Logger::Logger(SourceFile file, int line, LogLevel level) : impl_(level, 0, file, line) {}
Logger::Logger(SourceFile file, int line, LogLevel level, const char* func)
    : impl_(level, 0, file, line) {
  impl_.stream_ << '[' << func << "] ";
}
Logger::Logger(SourceFile file, int line, bool toAbort)
    : impl_(toAbort ? FATAL : ERROR, errno, file, line) {}

Logger::~Logger() {
  impl_.finish();
  const LogStream& stream = impl_.stream_;
  g_output(stream.buffer().data(), stream.buffer().length());
  if (impl_.level_ == FATAL) {
    g_flush();
    ::abort();
  }
}

Logger::LogLevel Logger::levelFromString(const char* name) {
  if (name == nullptr) return NUM_LOG_LEVELS;
  for (int lv = 0; lv < NUM_LOG_LEVELS; ++lv) {
    // kLogLevelNames 带 1 个尾空格，按前缀比较并核对长度
    size_t len = ::strlen(kLogLevelNames[lv]);
    while (len > 0 && kLogLevelNames[lv][len - 1] == ' ') --len;
    if (::strncasecmp(name, kLogLevelNames[lv], len) == 0 && ::strlen(name) == len) {
      return static_cast<LogLevel>(lv);
    }
  }
  return NUM_LOG_LEVELS;
}

namespace detail {

void defaultOutput(const char* msg, int len) {
  size_t n = ::fwrite(msg, 1, static_cast<size_t>(len), stdout);
  (void)n;
  ::fflush(stdout);
}

void defaultFlush() { ::fflush(stdout); }

}  // namespace detail

}  // namespace mhost