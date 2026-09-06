#include "LogStream.h"

#include <cstdio>

namespace mhost {

// ---- 整数 ----

LogStream::self& LogStream::operator<<(short v) {
  formatInteger(v);
  return *this;
}
LogStream::self& LogStream::operator<<(unsigned short v) {
  formatInteger(v);
  return *this;
}
LogStream::self& LogStream::operator<<(int v) {
  formatInteger(v);
  return *this;
}
LogStream::self& LogStream::operator<<(unsigned int v) {
  formatInteger(v);
  return *this;
}
LogStream::self& LogStream::operator<<(long v) {
  formatInteger(v);
  return *this;
}
LogStream::self& LogStream::operator<<(unsigned long v) {
  formatInteger(v);
  return *this;
}
LogStream::self& LogStream::operator<<(long long v) {
  formatInteger(v);
  return *this;
}
LogStream::self& LogStream::operator<<(unsigned long long v) {
  formatInteger(v);
  return *this;
}

// ---- 指针（十六进制，前缀 0x）----

LogStream::self& LogStream::operator<<(const void* p) {
  if (buffer_.avail() >= kMaxNumericSize) {
    char tmp[kMaxNumericSize];
    size_t len = 0;
    tmp[len++] = '0';
    tmp[len++] = 'x';
    uintptr_t v = reinterpret_cast<uintptr_t>(p);
    if (v == 0) {
      tmp[len++] = '0';
    } else {
      // 每 4 位一个十六进制字符，从最高非零半字节开始
      int shift = 60;
      while (shift >= 0 && ((v >> shift) & 0xF) == 0) {
        shift -= 4;
      }
      static const char hexDigits[] = "0123456789abcdef";
      while (shift >= 0) {
        tmp[len++] = hexDigits[(v >> shift) & 0xF];
        shift -= 4;
      }
    }
    buffer_.append(tmp, len);
  }
  return *this;
}

// ---- 浮点：%.12g，写不下则丢弃 ----

LogStream::self& LogStream::operator<<(double v) {
  if (buffer_.avail() >= kMaxNumericSize) {
    char tmp[kMaxNumericSize];
    int len = ::snprintf(tmp, sizeof tmp, "%.12g", v);
    if (len > 0) {
      buffer_.append(tmp, static_cast<size_t>(len));
    }
  }
  return *this;
}

}  // namespace mhost
