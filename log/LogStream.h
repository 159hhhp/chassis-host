#pragma once
/// @file LogStream.h
/// @brief 日志格式化流：定长 FixedBuffer + operator<< 集合 + printf 风格片段 Fmt
///
/// 设计要点：
///  - FixedBuffer 是"写满即丢弃溢出数据"的定长缓冲，永不越界；
///    小缓冲(4KB)作 LogStream 的栈上格式化区，大缓冲(4MB)作 AsyncLogging 前台缓冲。
///  - 整数转换采用倒序查表 + 反转的经典实现，避免 snprintf 的 locale 开销。

#include <cassert>
#include <cstring>
#include <cstdint>
#include <string>

namespace mhost {
namespace detail {

constexpr int kSmallBuffer = 4 * 1024;
constexpr int kLargeBuffer = 4 * 1000 * 1000;  // 4MB，AsyncLogging 前台缓冲

/// 定长缓冲区：append 溢出部分直接丢弃（安全截断）。
template <int SIZE>
class FixedBuffer {
 public:
  FixedBuffer() : cur_(data_) {}

  void append(const char* buf, size_t len) {
    if (static_cast<size_t>(avail()) > len) {
      ::memcpy(cur_, buf, len);
      cur_ += len;
    }
  }

  const char* data() const { return data_; }
  int length() const { return static_cast<int>(cur_ - data_); }
  int avail() const { return static_cast<int>(end() - cur_); }

  void reset() { cur_ = data_; }
  void bzero() { ::memset(data_, 0, sizeof data_); }

 private:
  const char* end() const { return data_ + sizeof data_; }

  char data_[SIZE];
  char* cur_;
};

/// 十进制整数转字符串，返回写入长度（倒序查表 + 就地反转）。
template <typename T>
size_t convert(char buf[], T value) {
  // 以 9 为中心对称的数字表：zero[0..8]='9'..'1'，zero[9..18]='0'..'9'，
  // 使负余数（i%10 为负）也能经 zero[lsd] 直接取得字符。
  static const char digits[] = "9876543210123456789";
  static const char* zero = digits + 9;
  T i = value;
  char* p = buf;
  do {
    int lsd = static_cast<int>(i % 10);
    i /= 10;
    *p++ = zero[lsd];
  } while (i != 0);
  *p = '\0';
  if (p - buf > 1) {
    char* lo = buf;
    char* hi = p - 1;
    while (lo < hi) {
      char t = *lo;
      *lo = *hi;
      *hi = t;
      ++lo;
      --hi;
    }
  }
  return static_cast<size_t>(p - buf);
}

}  // namespace detail

/// 日志格式化流：写入栈上 FixedBuffer<kSmallBuffer>，由 Logger 负责输出。
class LogStream {
 public:
  typedef LogStream self;

  self& operator<<(bool v) {
    buffer_.append(v ? "1" : "0", 1);
    return *this;
  }
  self& operator<<(char v) {
    buffer_.append(&v, 1);
    return *this;
  }
  self& operator<<(signed char v) { return operator<<(static_cast<short>(v)); }
  self& operator<<(unsigned char v) { return operator<<(static_cast<unsigned short>(v)); }
  self& operator<<(const char* str) {
    if (str) {
      buffer_.append(str, ::strlen(str));
    } else {
      buffer_.append("(null)", 6);
    }
    return *this;
  }
  self& operator<<(const unsigned char* str) {
    return operator<<(reinterpret_cast<const char*>(str));
  }
  self& operator<<(const std::string& v) {
    buffer_.append(v.c_str(), v.size());
    return *this;
  }

  self& operator<<(short v);
  self& operator<<(unsigned short v);
  self& operator<<(int v);
  self& operator<<(unsigned int v);
  self& operator<<(long v);
  self& operator<<(unsigned long v);
  self& operator<<(long long v);
  self& operator<<(unsigned long long v);

  self& operator<<(const void* p);
  self& operator<<(float v) { return operator<<(static_cast<double>(v)); }
  self& operator<<(double v);

  void append(const char* data, size_t len) { buffer_.append(data, len); }
  const detail::FixedBuffer<detail::kSmallBuffer>& buffer() const { return buffer_; }
  void resetBuffer() { buffer_.reset(); }

 private:
  static const int kMaxNumericSize = 32;

  template <typename T>
  void formatInteger(T v) {
    if (buffer_.avail() >= kMaxNumericSize) {
      char tmp[kMaxNumericSize];
      size_t len = detail::convert(tmp, v);
      buffer_.append(tmp, len);
    }
  }

  detail::FixedBuffer<detail::kSmallBuffer> buffer_;
};

/// printf 风格格式化片段：LogStream& s << Fmt("%06d", n);
/// 仅支持单标量参数，fmt 与参数类型必须匹配。
class Fmt {
 public:
  template <typename T>
  Fmt(const char* fmt, T val) {
    static_assert(sizeof(T) <= sizeof(long long), "Fmt 仅支持标量类型");
    length_ = ::snprintf(buf_, sizeof buf_, fmt, val);
    assert(static_cast<size_t>(length_) < sizeof buf_);
  }

  const char* data() const { return buf_; }
  int length() const { return length_; }

 private:
  char buf_[48];
  int length_;
};

inline LogStream& operator<<(LogStream& s, const Fmt& fmt) {
  s.append(fmt.data(), static_cast<size_t>(fmt.length()));
  return s;
}

}  // namespace mhost
