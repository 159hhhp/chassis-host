#include "LogFile.h"

#include <cstdio>
#include <ctime>
#include <unistd.h>

#include "FileUtil.h"

namespace mhost {

LogFile::LogFile(const std::string& basename, size_t rollSize, bool threadSafe,
                 int flushInterval, int checkEveryN)
    : basename_(basename),
      rollSize_(rollSize),
      flushInterval_(flushInterval),
      checkEveryN_(checkEveryN),
      count_(0),
      mutex_(threadSafe ? new std::mutex : nullptr),
      file_(new detail::AppendFile(getLogFileName(basename_, &lastRoll_))),
      startOfPeriod_(0),
      lastRoll_(0),
      lastFlush_(0) {
  // lastRoll_ 已由 getLogFileName() 经出参写入当前时间，这里补上文件所属“天”
  startOfPeriod_ = lastRoll_ / kRollPerSeconds_ * kRollPerSeconds_;
}

LogFile::~LogFile() = default;

void LogFile::append(const char* logline, int len) {
  if (mutex_) {
    std::lock_guard<std::mutex> lock(*mutex_);
    append_unlocked(logline, len);
  } else {
    append_unlocked(logline, len);
  }
}

void LogFile::flush() {
  if (mutex_) {
    std::lock_guard<std::mutex> lock(*mutex_);
    file_->flush();
  } else {
    file_->flush();
  }
}

void LogFile::append_unlocked(const char* logline, int len) {
  file_->append(logline, static_cast<size_t>(len));

  if (file_->writtenBytes() > static_cast<off_t>(rollSize_)) {
    rollFile();  // 大小滚动
  } else {
    ++count_;
    if (count_ >= checkEveryN_) {
      count_ = 0;
      time_t now = ::time(nullptr);
      time_t thisPeriod = now / kRollPerSeconds_ * kRollPerSeconds_;
      if (thisPeriod != startOfPeriod_) {
        rollFile();  // 跨天滚动
      } else if (now - lastFlush_ > flushInterval_) {
        lastFlush_ = now;
        file_->flush();  // 周期性落盘
      }
    }
  }
}

void LogFile::rollFile() {
  time_t now = 0;
  std::string filename = getLogFileName(basename_, &now);
  time_t start = now / kRollPerSeconds_ * kRollPerSeconds_;

  if (now > lastRoll_) {
    lastRoll_ = now;
    lastFlush_ = now;
    startOfPeriod_ = start;
    file_.reset(new detail::AppendFile(filename));
  }
}

std::string LogFile::getLogFileName(const std::string& basename, time_t* now) {
  *now = ::time(nullptr);
  struct tm tm_time;
  ::localtime_r(now, &tm_time);

  char pidbuf[16];
  ::snprintf(pidbuf, sizeof pidbuf, ".%d", static_cast<int>(::getpid()));
  char hostbuf[64];
  if (::gethostname(hostbuf, sizeof hostbuf) != 0) {
    ::snprintf(hostbuf, sizeof hostbuf, "unknownhost");
  }
  std::string name(hostbuf);
  // 主机名中的 '.' 会影响按扩展名分拣，替换掉
  size_t dot = name.find('.');
  if (dot != std::string::npos) {
    name.erase(dot);
  }

  char timebuf[48];
  ::snprintf(timebuf, sizeof timebuf, ".%4d%02d%02d-%02d%02d%02d",
             tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday,
             tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);

  return basename + timebuf + name + pidbuf + ".log";
}

}  // namespace mhost
