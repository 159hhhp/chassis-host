#pragma once
/// @file Timestamp.h
/// @brief 微秒级 UTC 时间戳（日志行与遥测打点用）

#include <cstdint>
#include <string>

namespace mhost {

/// 微秒级时间戳，支持格式化输出 "YYYYMMDD HH:MM:SS.uuuuuu"（本地时区）。
class Timestamp {
 public:
  Timestamp() : microSecondsSinceEpoch_(0) {}
  explicit Timestamp(int64_t microSecondsSinceEpoch)
      : microSecondsSinceEpoch_(microSecondsSinceEpoch) {}

  /// 当前时刻
  static Timestamp now();
  static Timestamp invalid() { return Timestamp(); }

  static const int kMicroPerSecond = 1000 * 1000;

  int64_t microSecondsSinceEpoch() const { return microSecondsSinceEpoch_; }
  time_t secondsSinceEpoch() const {
    return static_cast<time_t>(microSecondsSinceEpoch_ / kMicroPerSecond);
  }

  /// 本地时区格式化字符串
  std::string toFormattedString(bool showMicroseconds = true) const;

  bool valid() const { return microSecondsSinceEpoch_ > 0; }

  bool operator<(Timestamp rhs) const { return microSecondsSinceEpoch_ < rhs.microSecondsSinceEpoch_; }
  bool operator>(Timestamp rhs) const { return microSecondsSinceEpoch_ > rhs.microSecondsSinceEpoch_; }
  bool operator<=(Timestamp rhs) const { return microSecondsSinceEpoch_ <= rhs.microSecondsSinceEpoch_; }
  bool operator>=(Timestamp rhs) const { return microSecondsSinceEpoch_ >= rhs.microSecondsSinceEpoch_; }
  bool operator==(Timestamp rhs) const { return microSecondsSinceEpoch_ == rhs.microSecondsSinceEpoch_; }
  bool operator!=(Timestamp rhs) const { return microSecondsSinceEpoch_ != rhs.microSecondsSinceEpoch_; }

 private:
  int64_t microSecondsSinceEpoch_;
};

/// 两时刻之差（秒，可正可负）
inline double timeDifference(Timestamp high, Timestamp low) {
  return static_cast<double>(high.microSecondsSinceEpoch() - low.microSecondsSinceEpoch()) /
         Timestamp::kMicroPerSecond;
}

/// 时刻 + 秒偏移
inline Timestamp addTime(Timestamp t, double seconds) {
  return Timestamp(t.microSecondsSinceEpoch() +
                   static_cast<int64_t>(seconds * Timestamp::kMicroPerSecond));
}

}  // namespace mhost
