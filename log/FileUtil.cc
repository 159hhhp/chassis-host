#include "FileUtil.h"

#include <cerrno>
#include <cstring>
#include <unistd.h>

namespace mhost {
namespace detail {

namespace {
/// strerror_r 的线程安全封装（GNU/XSI 两种变体统一处理）
const char* fileStrerror(int err) {
  thread_local char buf[64];
  return ::strerror_r(err, buf, sizeof buf);
}
}  // namespace

AppendFile::AppendFile(std::string filename) : fp_(::fopen(filename.c_str(), "ae")), written_(0) {
  // "e"：fd 带 O_CLOEXEC，防止 exec 后泄漏
  if (fp_ != nullptr) {
    ::setbuffer(fp_, buffer_, sizeof buffer_);
  }
}

AppendFile::~AppendFile() {
  if (fp_ != nullptr) {
    ::fclose(fp_);
  }
}

void AppendFile::append(const char* logline, size_t len) {
  size_t written = 0;
  while (written < len) {
    size_t remain = len - written;
    size_t n = write(logline + written, remain);
    if (n != remain) {
      int err = ferror(fp_);
      if (err != 0) {
        fprintf(stderr, "AppendFile::append() failed: %s\n", fileStrerror(err));
        clearerr(fp_);
      }
      break;
    }
    written += n;
  }
  written_ += static_cast<off_t>(written);
}

size_t AppendFile::write(const char* logline, size_t len) {
#undef fwrite_unlocked
  return ::fwrite_unlocked(logline, 1, len, fp_);
}

void AppendFile::flush() {
  if (fp_ != nullptr) {
    ::fflush(fp_);
  }
}

}  // namespace detail
}  // namespace mhost
