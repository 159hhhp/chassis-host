#pragma once
/// @file FileUtil.h
/// @brief 追加写文件封装（64KB 用户态缓冲 + fwrite_unlocked）

#include <cstdio>
#include <string>
#include <sys/types.h>

namespace mhost {
namespace detail {

/// 追加写文件：打开失败仅置错误标志（由调用方日志报告），append 无效。
class AppendFile {
 public:
  explicit AppendFile(std::string filename);
  ~AppendFile();

  /// 追加一段日志（内部处理部分写入与 EINTR 重试）
  void append(const char* logline, size_t len);
  void flush();
  off_t writtenBytes() const { return written_; }

 private:
  size_t write(const char* logline, size_t len);

  FILE* fp_;
  char buffer_[64 * 1024];  ///< 大缓冲减少 fwrite 的系统调用次数
  off_t written_;
};

}  // namespace detail
}  // namespace mhost
