#pragma once
/// @file OtaFlasher.h
/// @brief 烧录子进程管理：posix_spawnp 启动独立进程组（flash.py 连同它
///        带起的 CubeProgrammer/stm32flash 同组，强杀不漏），stdout/stderr
///        合流到非阻塞 pipe，pipe 挂 muduo Channel 回 EventLoop 逐行上抛。
///
/// 为什么不用 fork+exec：进程内有异步日志后台线程，裸 fork 后子进程
/// 只允许 exec 一类异步信号安全调用，不适合再跑复杂逻辑；posix_spawn
/// 系列由 libc 在内部处理妥当。
/// 线程约束：全部接口须在 EventLoop 线程调用（Channel 生命周期）。

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <muduo/net/Channel.h>
#include <muduo/net/EventLoop.h>

namespace mhost {
namespace ota {

class OtaFlasher {
 public:
  /// 子进程一行输出（已按 \r/\n 切行，空行不上抛）
  using LineCb = std::function<void(const std::string& line)>;
  /// 子进程退出：ok = 正常退出且退出码 0；exitCode 为 waitpid 的原始 status
  using ExitCb = std::function<void(bool ok, int exitStatus)>;

  explicit OtaFlasher(muduo::net::EventLoop* loop);
  ~OtaFlasher();  // 仍在运行则杀进程组并同步回收

  bool running() const { return pid_ > 0; }

  /// 启动 argv（argv[0] 经 PATH 解析）；已在运行返回 false。
  /// 事件回调在 loop 线程触发；ExitCb 保证恰好触发一次。
  bool start(const std::vector<std::string>& argv, LineCb onLine, ExitCb onExit);

  /// 杀整个进程组：先 TERM，0.5s 未退再 KILL；退出仍经 ExitCb 上抛（ok=false）
  void killGroup();

 private:
  void onReadable();
  void scheduleDetachPipe();  ///< 延后到当前 Channel 回调返回后再销毁 Channel
  void detachPipe();          ///< EOF/错误：拆 Channel、关 fd，转入回收
  void pollReap(int tries);   ///< 周期 waitpid(WNOHANG)，超限阻塞收尸兜底
  void finish(int exitStatus);

  muduo::net::EventLoop* loop_;
  pid_t pid_ = 0;             // 同时是进程组 id（spawn 时 SETPGROUP 0）
  int pipeFd_ = -1;
  std::unique_ptr<muduo::net::Channel> channel_;
  std::string lineBuf_;       // 跨 read 残留的半行
  bool detachScheduled_ = false;

  LineCb lineCb_;
  ExitCb exitCb_;
  bool exited_ = false;
};

}  // namespace ota
}  // namespace mhost
