#include "Timestamp.h"

#include <cstdio>
#include <ctime>
#include <sys/time.h>

namespace mhost {

Timestamp Timestamp::now() {
  struct timespec ts;
  ::clock_gettime(CLOCK_REALTIME, &ts);
  return Timestamp(static_cast<int64_t>(ts.tv_sec) * kMicroPerSecond + ts.tv_nsec / 1000);
}

std::string Timestamp::toFormattedString(bool showMicroseconds) const {
  char buf[64] = {0};
  time_t seconds = secondsSinceEpoch();
  struct tm tm_time;
  ::localtime_r(&seconds, &tm_time);
  if (showMicroseconds) {
    int micro = static_cast<int>(microSecondsSinceEpoch_ % kMicroPerSecond);
    ::snprintf(buf, sizeof buf, "%4d%02d%02d %02d:%02d:%02d.%06d",
               tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday,
               tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec, micro);
  } else {
    ::snprintf(buf, sizeof buf, "%4d%02d%02d %02d:%02d:%02d",
               tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday,
               tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);
  }
  return buf;
}

}  // namespace mhost
